#include "app/remote_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace app {
namespace {

std::int64_t clockNs(clockid_t c) {
    timespec t;
    clock_gettime(c, &t);
    return static_cast<std::int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

bool sameAddr(const sockaddr_in6& a, const sockaddr_in6& b) {
    return std::memcmp(&a.sin6_addr, &b.sin6_addr, sizeof a.sin6_addr) == 0 && a.sin6_port == b.sin6_port;
}

constexpr std::int64_t kParamsPeriodNs = 500000000;     // PARAMS to the PC every 0.5 s (and on HELLO)
constexpr std::int64_t kPeerSwitchNs = 1000000000;      // a new PC may take over after 1 s without good replies
constexpr int kMaxDrain = 256;                           // datagrams per tick (a backlog after a stall drains in ticks)

}

RemoteLink::RemoteLink(int port, int delayTicks, double linkTimeout, std::size_t logReplies)
    : port_(port), delay_(delayTicks), timeout_ns_(static_cast<std::int64_t>(linkTimeout * 1e9)), slots_(kSlots),
      sent_(kSlots), log_(logReplies, true) {}

RemoteLink::~RemoteLink() {
    if (fd_ >= 0) ::close(fd_);
}

bool RemoteLink::open(std::string* err) {
    fd_ = ::socket(AF_INET6, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) {
        if (err) *err = std::string("remote: socket: ") + std::strerror(errno);
        return false;
    }
    int off = 0, on = 1, tclass = 0xb8;    // DSCP EF: harmless where the network ignores it
    setsockopt(fd_, IPPROTO_IPV6, IPV6_V6ONLY, &off, sizeof off);         // IPv4 PCs too (v4-mapped)
    if (setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof on) != 0) {
        if (err) *err = std::string("remote: SO_TIMESTAMPNS: ") + std::strerror(errno);
        return false;
    }
    setsockopt(fd_, IPPROTO_IPV6, IPV6_TCLASS, &tclass, sizeof tclass);
    setsockopt(fd_, IPPROTO_IP, IP_TOS, &tclass, sizeof tclass);
    sockaddr_in6 a{};
    a.sin6_family = AF_INET6;
    a.sin6_addr = in6addr_any;
    a.sin6_port = htons(static_cast<std::uint16_t>(port_));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) {
        if (err) *err = "remote: bind udp " + std::to_string(port_) + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

void RemoteLink::setParams(const proto::Params& p) {
    params_ = p;
    params_.delay_ticks = static_cast<std::uint16_t>(delay_);
    params_.params_id = proto::paramsId(params_);
}

void RemoteLink::beginTick(std::uint32_t seq, std::int64_t nowNs) {
    seq_ = seq;
    now_ = nowNs;
    rt_offset_ = clockNs(CLOCK_REALTIME) - clockNs(CLOCK_MONOTONIC);   // kernel timestamps are CLOCK_REALTIME
    drain();
    if (has_peer_ && now_ >= next_params_) sendParams();
}

void RemoteLink::drain() {
    for (int i = 0; i < kMaxDrain; ++i) {
        std::uint8_t buf[256];
        sockaddr_in6 from{};
        iovec iov{buf, sizeof buf};
        alignas(cmsghdr) char ctrl[CMSG_SPACE(sizeof(timespec)) + 64];
        msghdr m{};
        m.msg_name = &from;
        m.msg_namelen = sizeof from;
        m.msg_iov = &iov;
        m.msg_iovlen = 1;
        m.msg_control = ctrl;
        m.msg_controllen = sizeof ctrl;
        const ssize_t n = ::recvmsg(fd_, &m, MSG_DONTWAIT);
        if (n < 0) break;                    // EAGAIN: drained (other errors: try again next tick)
        std::int64_t arrival = now_;         // fallback: when it was drained
        for (cmsghdr* c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
                timespec ts;
                std::memcpy(&ts, CMSG_DATA(c), sizeof ts);
                arrival = static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec - rt_offset_;
            }
        if ((m.msg_flags & MSG_TRUNC) || from.sin6_family != AF_INET6) {
            ++st_.bad_packets;
            continue;
        }
        handle(buf, static_cast<std::size_t>(n), from, arrival);
    }
}

void RemoteLink::handle(const std::uint8_t* d, std::size_t n, const sockaddr_in6& from, std::int64_t arrivalNs) {
    const std::uint32_t kind = proto::kindOf(d, n);
    if (kind == proto::kMagicHello) {
        const bool same = has_peer_ && sameAddr(from, peer_);
        const bool quiet = last_good_ < 0 || now_ - last_good_ > kPeerSwitchNs;
        if (!same && has_peer_ && !quiet) {  // another PC while this one answers: ignore it
            ++st_.foreign;
            return;
        }
        if (!same) {
            peer_ = from;
            has_peer_ = true;
            last_good_ = -1;
            ++st_.peer_changes;
        }
        sendParams();
        return;
    }
    if (kind != proto::kMagicReply) {
        ++st_.bad_packets;
        return;
    }
    if (!has_peer_ || !sameAddr(from, peer_)) {
        ++st_.foreign;
        return;
    }
    proto::Reply r;
    proto::unpackReply(d, n, &r);
    if (r.params_id != params_.params_id || !(r.flags & proto::kReplyParamsOk)) {
        ++st_.wrong_params;
        return;
    }
    const std::uint32_t age = seq_ - r.seq;  // ticks since the sample; a reply "from the future" wraps to huge
    const SentRec& sr = sent_[r.seq % kSlots];
    if (age >= kSlots || !sr.valid || sr.seq != r.seq) {
        ++st_.stale;
        return;
    }
    Slot& sl = slots_[r.seq % kSlots];
    if (sl.valid && sl.seq == r.seq) {
        ++st_.duplicates;
        return;
    }
    sl.seq = r.seq;
    sl.valid = true;
    sl.a = r.a_cmd;
    last_good_ = arrivalNs;
    ++st_.replies;
    ++w_replies_;
    const std::int64_t rtt = arrivalNs - sr.t_ns;
    if (rtt > st_.rtt_max_ns) st_.rtt_max_ns = rtt;
    if (rtt > w_rtt_max_) w_rtt_max_ = rtt;
    if (age > static_cast<std::uint32_t>(delay_)) {   // drained at tick seq_, needed at tick r.seq + delay
        ++st_.late;
        ++w_late_;
    }
    if (sr.engaged) {
        const double diff = std::fabs(r.a_cmd - sr.a_local);
        ++st_.engaged_replies;
        st_.sum_diff2 += diff * diff;
        if (diff > st_.max_abs_diff) st_.max_abs_diff = diff;
    }
    log_.push(ReplyRec{r.seq, seq_, rtt, r.a_cmd, sr.engaged ? sr.a_local : 0.0, r.pc_proc_ns, r.flags, sr.engaged});
}

void RemoteLink::sendParams() {
    next_params_ = now_ + kParamsPeriodNs;
    if (!has_peer_) return;
    std::uint8_t b[proto::kParamsSize];
    proto::packParams(params_, b);
    if (::sendto(fd_, b, sizeof b, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&peer_), sizeof peer_) !=
        static_cast<ssize_t>(sizeof b))
        ++st_.send_errors;
}

void RemoteLink::sendSample(proto::Sample s, double aLocal) {
    s.seq = seq_;
    s.params_id = params_.params_id;
    s.delay_ticks = static_cast<std::uint16_t>(delay_);
    SentRec& sr = sent_[seq_ % kSlots];
    sr.seq = seq_;
    sr.valid = true;
    sr.t_ns = s.t_ns;
    sr.engaged = (s.flags & proto::kSampleController) != 0;
    sr.a_local = aLocal;
    if (!has_peer_) return;
    std::uint8_t b[proto::kSampleSize];
    proto::packSample(s, b);
    if (::sendto(fd_, b, sizeof b, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&peer_), sizeof peer_) ==
        static_cast<ssize_t>(sizeof b))
        ++st_.samples_sent;
    else
        ++st_.send_errors;
}

bool RemoteLink::command(std::uint32_t seq, double* a) const {
    const Slot& sl = slots_[seq % kSlots];
    if (!sl.valid || sl.seq != seq) return false;
    *a = sl.a;
    return true;
}

bool RemoteLink::linkUp() const { return has_peer_ && last_good_ >= 0 && now_ - last_good_ <= timeout_ns_; }

std::string RemoteLink::peer() const {
    if (!has_peer_) return "none";
    char b[INET6_ADDRSTRLEN + 8];
    if (IN6_IS_ADDR_V4MAPPED(&peer_.sin6_addr))
        inet_ntop(AF_INET, &peer_.sin6_addr.s6_addr[12], b, sizeof b);
    else
        inet_ntop(AF_INET6, &peer_.sin6_addr, b, sizeof b);
    return std::string(b) + ":" + std::to_string(ntohs(peer_.sin6_port));
}

void RemoteLink::takeWindow(std::uint64_t* replies, std::int64_t* rttMaxNs, std::uint64_t* late) {
    *replies = w_replies_;
    *rttMaxNs = w_rtt_max_;
    *late = w_late_;
    w_replies_ = w_late_ = 0;
    w_rtt_max_ = 0;
}

void writeRemoteCsv(const RemoteLink& link, const std::string& path) {
    writeCsv(link.replies(), path, "seq,rx_tick,rtt_ns,a_cmd,a_local,pc_proc_ns,flags,engaged",
             [](std::ostream& o, const ReplyRec& r) {
                 char b[200];
                 std::snprintf(b, sizeof b, "%u,%u,%lld,%.9g,%.9g,%u,%u,%d", r.seq, r.rx_tick,
                               static_cast<long long>(r.rtt_ns), r.a_cmd, r.a_local, r.pc_proc_ns,
                               static_cast<unsigned>(r.flags), r.engaged ? 1 : 0);
                 o << b;
             });
}

runlog::Fields remoteMetaFields(const RemoteLink& link, int delayTicks, double ts) {
    const LinkStats& s = link.stats();
    std::vector<std::int64_t> rtt;
    rtt.reserve(link.replies().size());
    link.replies().forEach([&](const ReplyRec& r) { rtt.push_back(r.rtt_ns); });
    std::sort(rtt.begin(), rtt.end());
    auto pct = [&](double q) {
        if (rtt.empty()) return std::string("-");
        const std::size_t i = std::min(rtt.size() - 1, static_cast<std::size_t>(q * static_cast<double>(rtt.size() - 1)));
        char b[32];
        std::snprintf(b, sizeof b, "%.3f", rtt[i] * 1e-6);
        return std::string(b);
    };
    char b[64];
    auto num = [&](double v, const char* f) {
        std::snprintf(b, sizeof b, f, v);
        return std::string(b);
    };
    return {{"remote_peer", link.peer()},
            {"remote_delay_ticks", std::to_string(delayTicks)},
            {"remote_delay_ms", num(delayTicks * ts * 1e3, "%.1f")},
            {"remote_params_id", std::to_string(link.paramsId())},
            {"remote_samples_sent", std::to_string(s.samples_sent)},
            {"remote_send_errors", std::to_string(s.send_errors)},
            {"remote_replies", std::to_string(s.replies)},
            {"remote_late_replies", std::to_string(s.late)},
            {"remote_stale", std::to_string(s.stale)},
            {"remote_duplicates", std::to_string(s.duplicates)},
            {"remote_wrong_params", std::to_string(s.wrong_params)},
            {"remote_foreign", std::to_string(s.foreign)},
            {"remote_bad_packets", std::to_string(s.bad_packets)},
            {"remote_peer_changes", std::to_string(s.peer_changes)},
            {"remote_rtt_ms_p50", pct(0.5)},
            {"remote_rtt_ms_p99", pct(0.99)},
            {"remote_rtt_ms_p999", pct(0.999)},
            {"remote_rtt_ms_max", num(s.rtt_max_ns * 1e-6, "%.3f")},
            {"remote_engaged_replies", std::to_string(s.engaged_replies)},
            {"remote_max_abs_diff", num(s.max_abs_diff, "%.3g")},
            {"remote_rms_diff",
             num(s.engaged_replies ? std::sqrt(s.sum_diff2 / static_cast<double>(s.engaged_replies)) : 0.0, "%.3g")}};
}

}
