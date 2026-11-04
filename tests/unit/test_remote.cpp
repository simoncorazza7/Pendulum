// Stage 2b remote controller (docs/remote.md): the UDP packet layout (golden bytes shared with py/remote_ctrl.py)
// and BalanceRuntime / SwingUpRuntime with a remote command source. A fake PC runs core::pidStep on every sample,
// exactly as py/remote_ctrl.py does, and delivers its replies after a chosen number of ticks.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "check.h"
#include "common/config.h"
#include "common/remote_proto.h"
#include "hw/sim_hw.h"
#include "runtime/balance.h"
#include "runtime/params.h"
#include "runtime/remote.h"
#include "runtime/swingup.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;

std::string hex(const std::uint8_t* b, std::size_t n) {
    std::string s;
    char t[3];
    for (std::size_t i = 0; i < n; ++i) {
        std::snprintf(t, sizeof t, "%02x", b[i]);
        s += t;
    }
    return s;
}

std::map<std::string, std::string> golden() {
    std::map<std::string, std::string> g;
    std::ifstream f("tests/golden/remote_packets.txt");
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string k, v;
        is >> k >> v;
        g[k] = v;
    }
    return g;
}

// The values py/remote_ctrl.py --golden packs (keep in step).
proto::Params goldenParams() {
    proto::Params p;
    p.delay_ticks = 3;
    p.params_id = 0x12345678;
    p.ts = 0.002; p.kp = 62.78; p.ki = 96.75; p.kd = 6.556; p.a_max = 2.0; p.v_max = 0.3; p.v_taper = 0.25;
    p.i_max = 1.0; p.i_delay = 0.0; p.theta_ref = 0.0; p.vel_fc = 14.0; p.cart_kx = 0.25; p.cart_kv = 1.0;
    p.cart_theta_max = 0.0698; p.cart_ki = 0.05; p.cart_i_max = 0.0349;
    return p;
}
proto::Sample goldenSample() {
    proto::Sample s;
    s.flags = 1; s.seq = 123456; s.epoch = 7; s.params_id = 0x12345678; s.t_ns = 1234567890123LL; s.counts = -4797;
    s.pos_usteps = 6104; s.theta = 0.0123; s.omega = -0.456; s.x = -0.0789; s.v = 0.101; s.mode = 2; s.phase = 3;
    s.delay_ticks = 3; s.i0 = 0.0042;
    return s;
}
proto::Reply goldenReply() {
    proto::Reply r;
    r.flags = 3; r.seq = 123453; r.epoch = 7; r.params_id = 0x12345678; r.a_cmd = -1.25; r.pc_proc_ns = 54321;
    return r;
}

// The PC, as py/remote_ctrl.py: resets its PID when the epoch changes (integrator = the sample's i0), runs it on
// samples flagged controller, replies 0 otherwise. reply latency (ticks) per seq from `latency` (< 0: the reply never arrives).
class FakePc : public rt::RemoteSource {
public:
    FakePc(const core::PidParams& p, std::function<int(std::uint32_t)> latency) : pid_(p), latency_(std::move(latency)) {}

    void beginTick(std::uint32_t k) {
        seq_ = k;
        for (auto it = pending_.begin(); it != pending_.end();) {
            if (it->second.first <= k) {
                arrived_[it->first] = it->second.second;
                it = pending_.erase(it);
            } else {
                ++it;
            }
        }
    }
    void sample(const rt::TickRecord& r) { sample(r.controller, r.epoch, r.i0, r.theta, r.x, r.v, r.omega); }
    void sample(const rt::SwingRecord& r) { sample(r.controller, r.epoch, r.i0, r.theta, r.x, r.v, r.omega); }
    void sample(bool controller, std::uint32_t epoch, double i0, double theta, double x, double v, double omega) {
        if (epoch != epoch_) { st_ = core::PidState{}; st_.integral = i0; epoch_ = epoch; }
        double a = 0.0;
        if (controller) {
            const core::PidResult pr = core::cascadeStep(pid_, st_, theta, x, v, kTs, &omega);
            st_ = pr.state;
            a = pr.a_cmd;
        }
        const int lat = latency_(seq_);
        if (lat >= 0) pending_[seq_] = {seq_ + static_cast<std::uint32_t>(lat), a};
    }

    std::uint32_t seq() const override { return seq_; }
    bool command(std::uint32_t s, double* a) const override {
        const auto it = arrived_.find(s);
        if (it == arrived_.end()) return false;
        *a = it->second;
        return true;
    }
    bool linkUp() const override { return up; }
    bool up = true;

private:
    core::PidParams pid_;
    core::PidState st_;
    std::uint32_t epoch_ = 0, seq_ = 0;
    std::function<int(std::uint32_t)> latency_;
    std::map<std::uint32_t, std::pair<std::uint32_t, double>> pending_;
    std::map<std::uint32_t, double> arrived_;
};

hw::SimHwParams rigPlant() {                     // the identified rig pendulum (config plant.*)
    hw::SimHwParams p;
    p.w0 = 4.931;
    p.bJ = 0.024;
    p.coulomb = 0.0117;
    return p;
}

hw::SimHwParams longTrack() {                    // switches and end stops kilometres away
    hw::SimHwParams p = rigPlant();
    p.pos_switch_right = -1e8;
    p.pos_switch_left = 1e8;
    p.pos_end_right = -2e8;
    p.pos_end_left = 2e8;
    return p;
}

struct Rig {
    hw::SimHwParams hp;
    hw::SimHw sim;
    rt::BalanceConfig cfg;
    explicit Rig(const hw::SimHwParams& p = rigPlant()) : hp(p), sim(hp, kPi, 0.0, 0.0) {
        cfg.calib = hp.truth;
        cfg.calib.c_bottom = sim.trueCBottomRelative();
        const auto g = core::designGains(hp.w0, hp.bJ, 2 * hp.w0, 0.7, 0.5 * hp.w0);
        cfg.pid.kp = g.kp; cfg.pid.ki = g.ki; cfg.pid.kd = g.kd;
        cfg.pid.a_max = 5.0; cfg.pid.v_max = 0.8; cfg.pid.i_delay = 0.1; cfg.pid.i_max = 1.0;
        cfg.sup.axis_a = hp.axis_a; cfg.sup.axis_v = hp.axis_v;
    }
};

// Arm while hanging, lift to 2.3 deg and hold; let go once engaged; a push at k = 1200; stop at k = 1800.
void script(int k, rt::BalanceRuntime& rt, hw::SimHw& sim) {
    if (k == 10) rt.requestArm();
    if (k == 50) sim.holdAt(0.04);
    if (sim.held() && rt.mode() == core::Mode::Engaged && k < 1100) sim.release();
    if (k == 1200) sim.holdAt(sim.theta() + 0.01, 0.04);
    if (k == 1220) sim.release();
    if (k == 1800) rt.requestStop();
}

std::uint32_t lcg(std::uint32_t x) { return x * 1664525u + 1013904223u; }

}

TEST(remote_packets_match_the_python_golden_bytes) {
    const auto g = golden();
    CHECK(g.size() >= 4);
    std::uint8_t b[256];                                          // >= every packet (128 overflowed from v3 on)
    CHECK_EQ(proto::packHello(b), proto::kHelloSize);
    CHECK_EQ(hex(b, proto::kHelloSize), g.at("hello"));
    CHECK_EQ(proto::packParams(goldenParams(), b), proto::kParamsSize);
    CHECK_EQ(hex(b, proto::kParamsSize), g.at("params"));
    proto::Params p;
    CHECK(proto::unpackParams(b, proto::kParamsSize, &p));
    CHECK_EQ(p.params_id, 0x12345678u);
    CHECK_EQ(p.kd, 6.556);
    CHECK_EQ(p.cart_kv, 1.0);
    CHECK_EQ(p.cart_theta_max, 0.0698);
    CHECK_EQ(p.cart_i_max, 0.0349);
    CHECK_EQ(proto::packSample(goldenSample(), b), proto::kSampleSize);
    CHECK_EQ(hex(b, proto::kSampleSize), g.at("sample"));
    proto::Sample s;
    CHECK(proto::unpackSample(b, proto::kSampleSize, &s));
    CHECK_EQ(s.counts, -4797);
    CHECK_EQ(s.t_ns, 1234567890123LL);
    CHECK_EQ(s.omega, -0.456);
    CHECK_EQ(s.i0, 0.0042);
    CHECK_EQ(proto::packReply(goldenReply(), b), proto::kReplySize);
    CHECK_EQ(hex(b, proto::kReplySize), g.at("reply"));
    proto::Reply r;
    CHECK(proto::unpackReply(b, proto::kReplySize, &r));
    CHECK_EQ(r.a_cmd, -1.25);
    CHECK_EQ(r.pc_proc_ns, 54321u);
    CHECK(!proto::unpackReply(b, proto::kReplySize - 1, &r));     // wrong size
    b[4] = 3;                                                     // wrong version (3: before the cart integral)
    CHECK(!proto::unpackReply(b, proto::kReplySize, &r));
    CHECK(proto::paramsId(goldenParams()) == proto::paramsId(goldenParams()));
    proto::Params q = goldenParams();
    q.kp += 1e-9;
    CHECK(proto::paramsId(q) != proto::paramsId(goldenParams()));
}

// The point of the constant delay: whatever the reply latency (1..N ticks), the command applied at tick k is the
// PC's from sample k - N, i.e. exactly the local PID through the Stage 2a delay line of N samples.
TEST(remote_constant_delay_equals_the_local_delay_line_under_jitter) {
    const int N = 4;
    Rig a(longTrack()), b(longTrack());
    for (Rig* r : {&a, &b}) {                             // a long track: the angle-only PID's drift must not end it
        r->cfg.sup.x_min = -20.0;
        r->cfg.sup.x_max = 20.0;
    }
    a.cfg.delay_samples = N;
    rt::BalanceRuntime local(a.cfg);
    std::uint32_t x = 12345;
    FakePc pc(b.cfg.pid, [&](std::uint32_t) {
        x = lcg(x);
        return 1 + static_cast<int>((x >> 16) % N);      // 1..N ticks: jitter inside the budget
    });
    b.cfg.remote = &pc;
    b.cfg.remote_delay = N;
    rt::BalanceRuntime remote(b.cfg);
    int engaged = 0, misses = 0, diffs = 0, stopEvent = 0;
    double maxLocalDiff = 0;
    for (int k = 0; k < 2200; ++k) {
        script(k, local, a.sim);
        script(k, remote, b.sim);
        pc.beginTick(static_cast<std::uint32_t>(k));
        const rt::TickRecord ra = local.tick(a.sim, false);
        const rt::TickRecord rb = remote.tick(b.sim, false);
        pc.sample(rb);
        a.sim.advance(kTs);
        b.sim.advance(kTs);
        if (ra.a_cmd != rb.a_cmd || ra.mode != rb.mode) ++diffs;
        if (rb.mode == static_cast<int>(core::Mode::Engaged)) ++engaged;
        if (rb.mode == static_cast<int>(core::Mode::Stopping) && !stopEvent) stopEvent = rb.event;
        if (rb.remote_miss) ++misses;
        if (rb.controller) maxLocalDiff = std::max(maxLocalDiff, std::fabs(rb.a_local - ra.a_ctrl));
    }
    CHECK(engaged > 1400);                               // it balanced (engaged ~ k 285..1800, through the push)
    CHECK_EQ(stopEvent, static_cast<int>(core::Reason::ManualStop));
    CHECK_EQ(diffs, 0);                                   // bit-identical commands and modes
    CHECK_EQ(misses, 0);
    CHECK(maxLocalDiff > 0.1);                            // (the commands were not trivially zero)
}

TEST(remote_late_reply_holds_the_previous_command_and_counts) {
    Rig rig;
    std::uint32_t lateSeq = 0;
    FakePc pc(rig.cfg.pid, [&](std::uint32_t s) { return s == lateSeq ? 6 : 1; });
    rig.cfg.remote = &pc;
    rig.cfg.remote_delay = 2;
    rt::BalanceRuntime rt(rig.cfg);
    std::vector<rt::TickRecord> recs;
    for (int k = 0; k < 1000; ++k) {
        script(k, rt, rig.sim);
        pc.beginTick(static_cast<std::uint32_t>(k));
        const rt::TickRecord r = rt.tick(rig.sim, false);
        if (r.mode == static_cast<int>(core::Mode::Engaged) && lateSeq == 0 && k > 400) lateSeq = static_cast<std::uint32_t>(k);
        pc.sample(r);
        rig.sim.advance(kTs);
        recs.push_back(r);
    }
    CHECK(lateSeq > 0);
    const std::size_t need = lateSeq + 2;                 // the tick that needed the late command
    CHECK(recs[need].remote_miss);
    CHECK_EQ(recs[need].a_cmd, recs[need - 1].a_cmd);     // held
    CHECK(!recs[need + 1].remote_miss);
    CHECK_EQ(recs[need].mode, static_cast<int>(core::Mode::Engaged));   // one miss < remote_max_miss: still balancing
    CHECK_EQ(recs.back().mode, static_cast<int>(core::Mode::Engaged));
}

TEST(remote_missing_commands_abort_after_max_miss) {
    Rig rig;
    std::uint32_t gapFrom = 0;
    FakePc pc(rig.cfg.pid, [&](std::uint32_t s) { return (gapFrom && s >= gapFrom) ? -1 : 1; });
    rig.cfg.remote = &pc;
    rig.cfg.remote_delay = 2;
    rig.cfg.remote_max_miss = 5;
    rt::BalanceRuntime rt(rig.cfg);
    int stopAt = -1, firstMiss = -1, ev = 0;
    for (int k = 0; k < 1200; ++k) {
        script(k, rt, rig.sim);
        pc.beginTick(static_cast<std::uint32_t>(k));
        const rt::TickRecord r = rt.tick(rig.sim, false);
        if (r.mode == static_cast<int>(core::Mode::Engaged) && !gapFrom && k > 400) gapFrom = static_cast<std::uint32_t>(k);
        if (r.remote_miss && firstMiss < 0) firstMiss = k;
        if (r.mode == static_cast<int>(core::Mode::Stopping) && stopAt < 0) { stopAt = k; ev = r.event; }
        pc.sample(r);
        rig.sim.advance(kTs);
    }
    CHECK(firstMiss == static_cast<int>(gapFrom) + 2);
    CHECK_EQ(stopAt, firstMiss + 4);                      // misses 1..4 held, the 5th in a row aborts
    CHECK_EQ(ev, static_cast<int>(core::Reason::RemoteFault));
}

TEST(remote_link_down_refuses_arming_and_aborts_armed) {
    Rig rig;
    FakePc pc(rig.cfg.pid, [](std::uint32_t) { return 1; });
    pc.up = false;
    rig.cfg.remote = &pc;
    rig.cfg.remote_delay = 2;
    rt::BalanceRuntime rt(rig.cfg);
    int refused = 0, armedAt = -1, stopEvent = 0;
    for (int k = 0; k < 400; ++k) {
        if (k == 10 || k == 100) rt.requestArm();
        if (k == 50) pc.up = true;
        if (k == 200) pc.up = false;                      // the PC goes away while ARMED
        pc.beginTick(static_cast<std::uint32_t>(k));
        const rt::TickRecord r = rt.tick(rig.sim, false);
        if (r.event == static_cast<int>(core::Reason::RemoteFault) && r.mode == static_cast<int>(core::Mode::Idle)) ++refused;
        if (r.event == static_cast<int>(core::Reason::ArmedEvent)) armedAt = k;
        if (r.mode == static_cast<int>(core::Mode::Stopping) && !stopEvent) stopEvent = r.event;
        pc.sample(r);
        rig.sim.advance(kTs);
    }
    CHECK_EQ(refused, 1);                                 // k = 10, link down
    CHECK_EQ(armedAt, 100);
    CHECK_EQ(stopEvent, static_cast<int>(core::Reason::RemoteFault));
}

TEST(remote_config_is_checked) {
    Config cfg = Config::fromFile("config/pendctl.conf");
    CHECK(!rt::sessionFrom(cfg).remote_mode);
    cfg.setFromAssignment("balance.remote=true");
    const rt::SessionConfig sc = rt::sessionFrom(cfg);
    CHECK(sc.remote_mode);
    CHECK(sc.balance.remote_delay >= 1);
    Config c0 = cfg;
    c0.setFromAssignment("remote.delay_ticks=0");
    CHECK_THROWS(rt::sessionFrom(c0));
    Config c1 = cfg;
    c1.setFromAssignment("balance.swingup=true");                 // remote swing-up (2026-11-03)
    CHECK(rt::sessionFrom(c1).swingup_mode);
    Config c2 = cfg;
    c2.setFromAssignment("ctrl.delay_samples=2");
    CHECK_THROWS(rt::sessionFrom(c2));
}

// --- remote swing-up (./swingup.sh --remote): pumping on the Pi, the balance after the hand-over on the PC ---
namespace {

hw::SimHwParams swingTrack() {                   // the 2026-11-02 pivot on the rig track (as test_swingup.cpp)
    hw::SimHwParams p;
    const double span = 6104 / 13333.0;
    p.w0 = 4.97; p.bJ = 0.054; p.coulomb = 0.168;
    p.axis_a = 400000.0 / (2 * 13333.0); p.axis_v = 120000.0 / (2 * 13333.0);
    p.pos_switch_right = 1000;
    p.pos_switch_left = 1000 + span * 13333.0;
    p.pos_end_right = 1000 - 0.015 * 13333.0;
    p.pos_end_left = p.pos_switch_left + 0.015 * 13333.0;
    p.truth.pos_center = 1000 + 0.5 * span * 13333.0;
    return p;
}

rt::SwingUpConfig swingConfig(const hw::SimHwParams& p) {
    rt::SwingUpConfig c = rt::swingUpFrom(Config::fromFile("config/pendctl.conf"));
    c.calib.c_bottom = 12.37 - std::floor(12.37 + 2400.0);   // the hanging start: SimHw counts begin at 0
    c.calib.pos_center = p.truth.pos_center;
    c.e_ref = 0.003;                     // SimHw loses less energy on the last approach than the rig (test_swingup.cpp)
    return c;
}

struct SwingRun {
    hw::SimHw sim;
    rt::SwingUpRuntime rt;
    FakePc* pc;
    std::vector<rt::SwingRecord> recs;
    bool limit = false;
    SwingRun(const rt::SwingUpConfig& c, const hw::SimHwParams& p, FakePc* f) : sim(p, kPi, 0.0, 0.0), rt(c), pc(f) {
        sim.enableDriver(true);
        recs.reserve(40000);
    }
    void until(int n, const std::function<bool(int)>& done) {
        for (int k = 0; k < n && !done(k); ++k) {
            const auto t = static_cast<std::uint32_t>(recs.size());
            if (pc) pc->beginTick(t);
            const hw::Sample s = sim.read();
            limit = limit || s.limit_left || s.limit_right;
            const rt::SwingRecord r = rt.tick(sim, s, false);
            if (pc) pc->sample(r);
            sim.advance(kTs);
            recs.push_back(r);
        }
    }
    int handover() const {
        for (std::size_t k = 0; k < recs.size(); ++k)
            if (recs[k].phase == static_cast<int>(rt::SwingPhase::Balance)) return static_cast<int>(k);
        return -1;
    }
};

}

// The pumping stays local; from the hand-over tick k0 on, the command at tick k is the PC's from sample k - N:
// 0 for the first N ticks (the PC answered 0 to samples before the hand-over), then exactly the shadow PID's
// a_local of tick k - N, whatever the reply jitter within N. The PC starts from the Pi's pre-load (i0).
TEST(remote_swingup_balances_on_the_pc_with_the_constant_delay) {
    const int N = 6;
    const hw::SimHwParams p = swingTrack();
    rt::SwingUpConfig c = swingConfig(p);
    std::uint32_t x = 777;
    FakePc pc(c.pid, [&](std::uint32_t) {
        x = lcg(x);
        return 1 + static_cast<int>((x >> 16) % N);
    });
    c.remote = &pc;
    c.remote_delay = N;
    c.epoch_base = 4;
    SwingRun r(c, p, &pc);
    r.until(60000, [&](int) { return r.rt.done(); });
    const int k0 = r.handover();
    CHECK(k0 > 0);
    CHECK(r.rt.handedOver());
    CHECK(r.rt.balanceTime() > 1.0);
    CHECK_EQ(r.recs[static_cast<std::size_t>(k0)].epoch, 5u);
    CHECK(std::fabs(r.recs[static_cast<std::size_t>(k0)].i0) > 1e-4);          // the pre-load went with the samples
    int balance = 0, mismatch = 0, misses = 0;
    double maxA = 0;
    for (std::size_t k = static_cast<std::size_t>(k0); k < r.recs.size(); ++k) {
        const rt::SwingRecord& w = r.recs[k];
        if (w.phase != static_cast<int>(rt::SwingPhase::Balance)) break;
        ++balance;
        if (w.remote_miss) ++misses;
        const double want = k < static_cast<std::size_t>(k0 + N) ? 0.0 : r.recs[k - N].a_local;
        if (w.a_cmd != want) ++mismatch;
        maxA = std::max(maxA, std::fabs(w.a_cmd));
    }
    CHECK(balance > 500);
    CHECK_EQ(mismatch, 0);
    CHECK_EQ(misses, 0);
    CHECK(maxA > 0.1);
    CHECK(r.rt.done());
    CHECK(!r.limit);
    CHECK(!r.sim.crashed());
    std::printf("    remote swing-up: hand-over %.2f s, balanced %.2f s (%s), %d BALANCE ticks bit-identical to the "
                "shadow PID %d ticks earlier\n", r.rt.handoverTime(), r.rt.balanceTime(), r.rt.reason(), balance, N);
}

// The PC goes silent while balancing: max_miss ticks later the run brakes ("remote link") and the energy is
// pumped out locally, down to a quiet hanging swing.
TEST(remote_swingup_link_loss_brakes_and_brings_it_down) {
    const hw::SimHwParams p = swingTrack();
    rt::SwingUpConfig c = swingConfig(p);
    std::uint32_t silentFrom = 0;
    FakePc pc(c.pid, [&](std::uint32_t s) { return (silentFrom && s >= silentFrom) ? -1 : 1; });
    c.remote = &pc;
    c.remote_delay = 6;
    c.remote_max_miss = 5;
    SwingRun r(c, p, &pc);
    r.until(60000, [&](int) {
        const int k0 = r.handover();
        if (k0 >= 0 && !silentFrom && static_cast<int>(r.recs.size()) > k0 + 100)
            silentFrom = static_cast<std::uint32_t>(r.recs.size());
        return r.rt.done();
    });
    CHECK(silentFrom > 0);
    CHECK_EQ(std::string(r.rt.reason()), std::string("remote link"));
    CHECK(r.rt.done());
    CHECK(r.recs.back().energy < 0.1);                                         // brought down
    CHECK(!r.limit);
    CHECK(!r.sim.crashed());
}

// No PC while pumping: the run never hands over into a dead link; it removes the energy instead.
TEST(remote_swingup_link_down_while_pumping_removes_energy) {
    const hw::SimHwParams p = swingTrack();
    rt::SwingUpConfig c = swingConfig(p);
    FakePc pc(c.pid, [](std::uint32_t) { return 1; });
    c.remote = &pc;
    c.remote_delay = 6;
    SwingRun r(c, p, &pc);
    r.until(60000, [&](int k) {
        if (k == 2500) pc.up = false;                                          // 5 s into the pumping
        return r.rt.done();
    });
    CHECK_EQ(std::string(r.rt.reason()), std::string("remote link down"));
    CHECK(!r.rt.handedOver());
    CHECK(r.rt.done());
    CHECK(!r.limit);
    CHECK(!r.sim.crashed());
}

// Stage 3b hand-over gate (swingup.v_sw): the hand-over happens only at a tick where the predicted post-catch cart
// speed |v + (g/w0)(theta_f + theta'/w0)| is inside the gate. During the approach that prediction sweeps (the cart
// accelerates), so the gate picks the moment in the window; a tighter gate never hands over earlier, and with the
// gate off (0) the first tick in the angle window is taken.
TEST(swingup_speed_gate_picks_a_slow_hand_over) {
    const hw::SimHwParams p = swingTrack();
    auto handover = [&](double vsw, double* vPred) {
        rt::SwingUpConfig c = swingConfig(p);
        c.v_sw = vsw;
        SwingRun r(c, p, nullptr);
        r.until(40000, [&](int) { return r.rt.done() || r.handover() >= 0; });
        const int k0 = r.handover();
        if (k0 < 0) return -1.0;
        const rt::SwingRecord& w = r.recs[static_cast<std::size_t>(k0)];
        *vPred = w.v + c.g / c.w0 * (w.theta_f + w.omega / c.w0);
        return k0 * kTs;
    };
    double vOff = 0, vCfg = 0, vTight = 0;
    const rt::SwingUpConfig base = swingConfig(p);
    CHECK(base.v_sw > 0.0);                                // the config default has the gate on
    const double tOff = handover(0.0, &vOff), tCfg = handover(base.v_sw, &vCfg), tTight = handover(0.01, &vTight);
    std::printf("    hand-over: gate off %.2f s (v_pred %+.0f mm/s), %.2f m/s %.2f s (%+.0f), 0.01 m/s %.2f s (%+.0f)\n",
                tOff, vOff * 1e3, base.v_sw, tCfg, vCfg * 1e3, tTight, vTight * 1e3);
    CHECK(tOff > 0 && tCfg > 0);
    CHECK(std::fabs(vCfg) < base.v_sw);
    CHECK(tCfg >= tOff);
    if (tTight > 0) {                                      // it may also never come
        CHECK(std::fabs(vTight) < 0.01);
        CHECK(tTight >= tCfg);
    }
}
