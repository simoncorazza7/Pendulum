#ifndef PENDEL_APP_REMOTE_LINK_H
#define PENDEL_APP_REMOTE_LINK_H

#include <netinet/in.h>

#include <cstdint>
#include <string>
#include <vector>

#include "common/remote_proto.h"
#include "common/runlog.h"
#include "rt/ringlog.h"
#include "runtime/remote.h"

// Pi side of the Stage 2b remote controller (docs/remote.md, protocol in common/remote_proto.h). One
// non-blocking UDP socket (dual-stack, port remote.port), used from the control loop only:
//   beginTick(seq)   drain the replies that arrived (kernel receive timestamps: the round trip is exact, not
//                    rounded to ticks), keep them by seq; then this tick's command is command(seq - delay)
//   sendSample(...)  stream the tick's sample to the PC that sent the last HELLO (PARAMS on HELLO and every 0.5 s)
// The PC's replies are only used if they come from that PC, carry the current params_id and are not from the
// future. Everything is counted (stats) and every reply is logged (replies(): remote.csv).
namespace app {

struct ReplyRec {
    std::uint32_t seq = 0;          // sample the PC computed from
    std::uint32_t rx_tick = 0;      // loop tick that drained it (usable from this tick on)
    std::int64_t rtt_ns = 0;        // arrival (kernel timestamp) - sample time (both CLOCK_MONOTONIC)
    double a_cmd = 0;               // m/s^2
    double a_local = 0;             // the Pi's shadow PID on the same sample (0 when not engaged)
    std::uint32_t pc_proc_ns = 0;   // PC: receive -> send
    std::uint16_t flags = 0;        // REPLY flags
    bool engaged = false;           // the sample was an ENGAGED one
};

struct LinkStats {
    std::uint64_t samples_sent = 0, send_errors = 0, replies = 0, bad_packets = 0, foreign = 0, stale = 0,
                  wrong_params = 0, duplicates = 0, late = 0;   // late: arrived after the tick that needed it
    std::uint64_t engaged_replies = 0;
    double max_abs_diff = 0;        // m/s^2, max |a_remote - a_local| over ENGAGED samples
    double sum_diff2 = 0;
    std::int64_t rtt_max_ns = 0;
    std::uint64_t peer_changes = 0;
};

class RemoteLink : public rt::RemoteSource {
public:
    RemoteLink(int port, int delayTicks, double linkTimeout, std::size_t logReplies);
    ~RemoteLink() override;
    RemoteLink(const RemoteLink&) = delete;
    RemoteLink& operator=(const RemoteLink&) = delete;

    bool open(std::string* err);
    void setParams(const proto::Params& p);              // params_id is computed here
    std::uint32_t paramsId() const { return params_.params_id; }

    void beginTick(std::uint32_t seq, std::int64_t nowNs);
    // s.seq / params_id / delay_ticks are filled in. aLocal: the shadow PID's output on this sample (ENGAGED).
    void sendSample(proto::Sample s, double aLocal);

    // rt::RemoteSource
    std::uint32_t seq() const override { return seq_; }
    bool command(std::uint32_t seq, double* a) const override;
    bool linkUp() const override;

    bool hasPeer() const { return has_peer_; }
    std::string peer() const;
    const LinkStats& stats() const { return st_; }
    const RingLog<ReplyRec>& replies() const { return log_; }
    // Replies, worst round trip and late replies since the last call (status lines).
    void takeWindow(std::uint64_t* replies, std::int64_t* rttMaxNs, std::uint64_t* late);

private:
    void drain();
    void handle(const std::uint8_t* d, std::size_t n, const sockaddr_in6& from, std::int64_t arrivalNs);
    void sendParams();

    static constexpr std::size_t kSlots = 4096;      // seq ring (8 s at 2 ms): replies and sample records
    struct Slot { std::uint32_t seq = 0; bool valid = false; double a = 0; };
    struct SentRec { std::uint32_t seq = 0; bool valid = false; std::int64_t t_ns = 0; double a_local = 0; bool engaged = false; };

    int port_, delay_;
    std::int64_t timeout_ns_;
    int fd_ = -1;
    proto::Params params_;
    bool has_peer_ = false;
    sockaddr_in6 peer_{};
    std::int64_t now_ = 0, last_good_ = -1, rt_offset_ = 0, next_params_ = 0;
    std::uint32_t seq_ = 0;
    std::vector<Slot> slots_;
    std::vector<SentRec> sent_;
    RingLog<ReplyRec> log_;
    LinkStats st_;
    std::uint64_t w_replies_ = 0, w_late_ = 0;
    std::int64_t w_rtt_max_ = 0;
};

// remote.csv: one row per accepted reply.
void writeRemoteCsv(const RemoteLink& link, const std::string& path);
// meta.json fields: link counters, round-trip percentiles (over the logged replies), late replies per delay.
runlog::Fields remoteMetaFields(const RemoteLink& link, int delayTicks, double ts);

}

#endif
