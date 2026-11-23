#include "runtime/actuator.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace rt {

namespace {

const double kPi = 3.141592653589793;

// Out-and-back pair, centred: -d/2 -> +d/2, hold, +d/2 -> -d/2 starting n T0 after the first leg.
class PairTraj : public ActTraj {
public:
    PairTraj(double d, double v, double a, double T0) : d_(d), out_(d, v, a), back_(-d, v, a) {
        t_ret_ = std::max(1.0, std::ceil(out_.duration() / T0 - 1e-9)) * T0;
    }
    core::ProfilePoint at(double t) const override {
        core::ProfilePoint p = t < t_ret_ ? out_.at(t) : back_.at(t - t_ret_);
        p.x += t < t_ret_ ? -0.5 * d_ : 0.5 * d_;
        return p;
    }
    double duration() const override { return t_ret_ + back_.duration(); }

private:
    double d_, t_ret_ = 0;
    core::TrapezoidProfile out_, back_;
};

// One trapezoid leg from x0 over the signed distance d.
class LegTraj : public ActTraj {
public:
    LegTraj(double x0, double d, double v, double a) : x0_(x0), p_(d, v, a) {}
    core::ProfilePoint at(double t) const override {
        core::ProfilePoint p = p_.at(t);
        p.x += x0_;
        return p;
    }
    double duration() const override { return p_.duration(); }

private:
    double x0_;
    core::TrapezoidProfile p_;
};

// +-a square wave: +a for P/4, then an odd number of half periods -a, +a, ..., -a, then +a for P/4, with
// P = 4 v / a. The speed swings between +-v, x between +-v P / 8 about the centre, and it ends at rest
// where it started (x = -v P / 8).
class BangTraj : public ActTraj {
public:
    BangTraj(double v, double a, double seconds) {
        const double P = 4 * v / a;
        int nHalf = static_cast<int>(std::floor(seconds / (0.5 * P) + 1e-9)) - 1;
        if (nHalf % 2 == 0) --nHalf;
        nHalf = std::max(nHalf, 1);
        double t = 0, x = -v * P / 8, vel = 0;
        auto add = [&](double acc, double dur) {
            seg_.push_back({t, x, vel, acc});
            x += vel * dur + 0.5 * acc * dur * dur;
            vel += acc * dur;
            t += dur;
        };
        add(a, 0.25 * P);
        for (int i = 0; i < nHalf; ++i) add(i % 2 == 0 ? -a : a, 0.5 * P);
        add(a, 0.25 * P);
        T_ = t;
        x_end_ = x;
    }
    core::ProfilePoint at(double t) const override {
        if (t <= 0) return {seg_.front().x, 0.0, 0.0};
        if (t >= T_) return {x_end_, 0.0, 0.0};
        std::size_t i = static_cast<std::size_t>(
            std::upper_bound(seg_.begin(), seg_.end(), t, [](double tt, const Seg& s) { return tt < s.t; }) - seg_.begin());
        const Seg& s = seg_[i - 1];
        const double dt = t - s.t;
        return {s.x + s.v * dt + 0.5 * s.a * dt * dt, s.v + s.a * dt, s.a};
    }
    double duration() const override { return T_; }

private:
    struct Seg { double t, x, v, a; };
    std::vector<Seg> seg_;
    double T_ = 0, x_end_ = 0;
};

// x = w(t) X(t) sin(phi(t)): exponential sweep f0 -> f1 over T, X = A / (2 pi f)^2 (acceleration amplitude
// A), w = cosine ramps of r s at both ends. v and a by central differences of x.
class ChirpTraj : public ActTraj {
public:
    ChirpTraj(double A, double f0, double f1, double T, double r) : A_(A), f0_(f0), T_(T), r_(r) {
        k_ = std::log(f1 / f0) / T;
    }
    core::ProfilePoint at(double t) const override {
        const double h = 1e-4;
        const double x0 = xf(t), xm = xf(t - h), xp = xf(t + h);
        if (t <= 0 || t >= T_) return {0.0, 0.0, 0.0};
        return {x0, (xp - xm) / (2 * h), (xp - 2 * x0 + xm) / (h * h)};
    }
    double duration() const override { return T_; }

private:
    double xf(double t) const {
        if (t <= 0 || t >= T_) return 0.0;
        const double e = std::exp(k_ * t);
        const double w2 = 2 * kPi * f0_ * e;                 // rad/s
        const double phi = 2 * kPi * f0_ * (e - 1) / k_;
        double w = 1.0;
        if (t < r_) w = 0.5 * (1 - std::cos(kPi * t / r_));
        else if (t > T_ - r_) w = 0.5 * (1 - std::cos(kPi * (T_ - t) / r_));
        return w * A_ / (w2 * w2) * std::sin(phi);
    }
    double A_, f0_, T_, r_, k_ = 0;
};

// A reference table (every ts, x from the centre), linearly interpolated; it ends where its last row is.
class TableTraj : public ActTraj {
public:
    TableTraj(const std::vector<core::ProfilePoint>* p, double ts) : p_(p), ts_(ts) {}
    core::ProfilePoint at(double t) const override {
        const std::vector<core::ProfilePoint>& p = *p_;
        if (p.empty()) return {0.0, 0.0, 0.0};
        if (t <= 0) return p.front();
        const double u = t / ts_;
        const std::size_t i = static_cast<std::size_t>(u);
        if (i + 1 >= p.size()) return {p.back().x, 0.0, 0.0};
        const double f = u - static_cast<double>(i);
        const core::ProfilePoint& a = p[i];
        const core::ProfilePoint& b = p[i + 1];
        return {a.x + f * (b.x - a.x), a.v + f * (b.v - a.v), a.a + f * (b.a - a.a)};
    }
    double duration() const override { return p_->size() > 1 ? static_cast<double>(p_->size() - 1) * ts_ : 0.0; }

private:
    const std::vector<core::ProfilePoint>* p_;
    double ts_;
};

struct Peaks { double x = 0, v = 0, a = 0; };

Peaks peaksOf(const ActTraj& t, double ts) {
    Peaks p;
    const int n = static_cast<int>(std::ceil(t.duration() / ts));
    for (int k = 0; k <= n; ++k) {
        const core::ProfilePoint q = t.at(k * ts);
        p.x = std::max(p.x, std::fabs(q.x));
        p.v = std::max(p.v, std::fabs(q.v));
        p.a = std::max(p.a, std::fabs(q.a));
    }
    return p;
}

// Lowest start frequency (Hz) that keeps the chirp within |v| <= v and |x| <= stream_x_max; 0 if none below f1.
double chirpStart(const ActuatorConfig& c, double v, double a) {
    double f0 = std::max(a / (2 * kPi * v), std::sqrt(a / c.stream_x_max) / (2 * kPi));
    for (int i = 0; i < 80 && f0 < c.chirp_f1; ++i) {
        const Peaks p = peaksOf(ChirpTraj(a, f0, c.chirp_f1, c.stream_seconds, c.chirp_taper), c.ts);
        if (p.v <= v && p.x <= c.stream_x_max) return f0;
        f0 *= 1.05;
    }
    return 0.0;
}

double legLength(const ActuatorConfig& c, double v, double a) { return v * v / a + v * c.cruise; }

}

const char* actProfileName(ActProfile p) {
    switch (p) {
        case ActProfile::Pair: return "pair";
        case ActProfile::OneWay: return "oneway";
        case ActProfile::Bang: return "bang";
        case ActProfile::Chirp: return "chirp";
        case ActProfile::Table: return "table";
    }
    return "?";
}

bool parseActProfile(const char* s, ActProfile* p) {
    for (ActProfile q : {ActProfile::Pair, ActProfile::OneWay, ActProfile::Bang, ActProfile::Chirp, ActProfile::Table})
        if (std::strcmp(s, actProfileName(q)) == 0) { *p = q; return true; }
    return false;
}

const char* actPhaseName(ActPhase p) {
    switch (p) {
        case ActPhase::WaitGo: return "WAIT_GO";
        case ActPhase::Home: return "HOME";
        case ActPhase::Ready: return "READY";
        case ActPhase::ToStart: return "TO_START";
        case ActPhase::Settle: return "SETTLE";
        case ActPhase::Run: return "RUN";
        case ActPhase::Brake: return "BRAKE";
        case ActPhase::Touch: return "TOUCH";
        case ActPhase::Centre: return "CENTRE";
        case ActPhase::Finishing: return "FINISHING";
        case ActPhase::Finished: return "FINISHED";
        case ActPhase::Failed: return "FAILED";
    }
    return "?";
}

const char* verdictName(Verdict v) {
    switch (v) {
        case Verdict::NotRun: return "not run";
        case Verdict::Pass: return "PASS";
        case Verdict::Marginal: return "marginal";
        case Verdict::Fail: return "FAIL";
        case Verdict::Aborted: return "aborted";
        case Verdict::Skipped: return "skipped";
    }
    return "?";
}

std::unique_ptr<ActTraj> makeActTraj(const ActuatorConfig& c, const ActRung& r, int rep) {
    switch (c.profile) {
        case ActProfile::Pair:
            return std::make_unique<PairTraj>(legLength(c, r.v, r.a), r.v, r.a, 2 * kPi / c.w0);
        case ActProfile::OneWay: {
            const double d = legLength(c, r.v, r.a), dir = rep % 2 == 1 ? 1.0 : -1.0;   // odd reps toward +x
            return std::make_unique<LegTraj>(-0.5 * dir * d, dir * d, r.v, r.a);
        }
        case ActProfile::Bang:
            return std::make_unique<BangTraj>(r.v, r.a, c.stream_seconds);
        case ActProfile::Chirp:
            if (!(r.f0 > 0)) return nullptr;
            return std::make_unique<ChirpTraj>(r.a, r.f0, c.chirp_f1, c.stream_seconds, c.chirp_taper);
        case ActProfile::Table:
            return std::make_unique<TableTraj>(&c.table, c.ts);
    }
    return nullptr;
}

bool loadActTable(const std::string& path, double ts, std::vector<core::ProfilePoint>* out, std::string* err) {
    std::ifstream f(path);
    if (!f) { *err = "cannot open " + path; return false; }
    out->clear();
    std::string line;
    bool header = false;
    double tPrev = 0;
    int n = 0;
    while (std::getline(f, line)) {
        ++n;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        if (!header) {
            if (line != "t,x,v,a") { *err = path + ": the header must be t,x,v,a"; return false; }
            header = true;
            continue;
        }
        double f[4];
        const char* p = line.c_str();
        bool ok = true;
        for (int k = 0; k < 4 && ok; ++k) {
            char* end = nullptr;
            f[k] = std::strtod(p, &end);
            ok = end != p && std::isfinite(f[k]) && *end == (k < 3 ? ',' : '\0');
            p = end + (k < 3 ? 1 : 0);
        }
        if (!ok) {
            *err = path + ":" + std::to_string(n) + ": expected four numbers";
            return false;
        }
        const double t = f[0], x = f[1], v = f[2], a = f[3];
        if (!out->empty() && std::fabs(t - tPrev - ts) > 1e-6) {
            *err = path + ":" + std::to_string(n) + ": rows must be loop.ts apart";
            return false;
        }
        tPrev = t;
        out->push_back({x, v, a});
    }
    if (out->size() < 2) { *err = path + ": fewer than two rows"; return false; }
    return true;
}

std::vector<ActRung> buildActRungs(const ActuatorConfig& c) {
    std::vector<ActRung> out;
    if (c.profile == ActProfile::Table) {
        ActRung r;
        r.runnable = c.table.size() > 1;
        if (!r.runnable) r.why = "the table is empty";
        else {
            const Peaks p = peaksOf(TableTraj(&c.table, c.ts), c.ts);
            r.v = r.peak_v = p.v;
            r.a = r.peak_a = p.a;
            r.leg = p.x;
            r.duration = static_cast<double>(c.table.size() - 1) * c.ts;
        }
        out.push_back(r);
        return out;
    }
    for (std::size_t i = 0; i < c.speeds.size(); ++i) {
        for (double a : c.accels) {
            ActRung r;
            r.v = c.speeds[i];
            r.a = a;
            r.speed_index = static_cast<int>(i);
            if (c.profile == ActProfile::Pair || c.profile == ActProfile::OneWay) {
                r.leg = legLength(c, r.v, r.a);
                if (r.leg > c.leg_max) { r.runnable = false; r.why = "leg longer than actuator.leg_max"; }
            } else if (c.profile == ActProfile::Chirp) {
                r.f0 = chirpStart(c, r.v, r.a);
                if (!(r.f0 > 0)) { r.runnable = false; r.why = "the chirp would have to start above actuator.chirp_f1"; }
            }
            if (r.runnable) {
                const std::unique_ptr<ActTraj> t = makeActTraj(c, r, 1);
                const Peaks p = peaksOf(*t, c.ts);
                r.duration = t->duration();
                r.peak_v = p.v;
                r.peak_a = p.a;
                if (c.profile != ActProfile::Pair && c.profile != ActProfile::OneWay) r.leg = p.x;
            }
            out.push_back(r);
        }
    }
    return out;
}

ActuatorTest::ActuatorTest(const ActuatorConfig& c) : c_(c), mover_(c.mover), calib_(c.calib) {
    rungs_ = buildActRungs(c_);
    rung_verdict_.assign(rungs_.size(), Verdict::NotRun);
    for (std::size_t i = 0; i < rungs_.size(); ++i)
        if (!rungs_[i].runnable) rung_verdict_[i] = Verdict::Skipped;
    results_.reserve(1000);
    window_.assign(std::max<std::size_t>(1, static_cast<std::size_t>(std::llround(2 * kPi / c_.w0 / c_.ts))), 0);
}

int ActuatorTest::nextRung() const {
    for (std::size_t i = 0; i < rung_verdict_.size(); ++i)
        if (rung_verdict_[i] == Verdict::NotRun) return static_cast<int>(i);
    return -1;
}

void ActuatorTest::driver(hw::Hardware& hw, bool on) {
    if (on != driver_on_) {
        if (!on) hw.commandAccel(0);
        hw.enableDriver(on);
        driver_on_ = on;
    }
}

void ActuatorTest::failWith(hw::Hardware& hw, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(reason_, sizeof reason_, fmt, ap);
    va_end(ap);
    driver(hw, false);
    phase_ = ActPhase::Failed;
}

std::int64_t ActuatorTest::posAt(double x) const {
    return centre_ + std::llround(calib_.x_sign * x * calib_.usteps_per_m);
}

double ActuatorTest::hold(hw::Hardware& hw, double v) {
    const double a = v != 0.0 ? -std::copysign(std::min(c_.hold_a, std::fabs(v) / c_.ts), v) : 0.0;
    hw.commandAccel(core::axisAccelArg(a, calib_));
    return a;
}

void ActuatorTest::updateSwing(std::int64_t counts) {
    const std::size_t n = window_.size();
    window_[win_i_] = counts;
    win_i_ = (win_i_ + 1) % n;
    win_n_ = std::min(win_n_ + 1, n);
    std::int64_t lo = counts, hi = counts;
    for (std::size_t i = 0; i < win_n_; ++i) {
        lo = std::min(lo, window_[i]);
        hi = std::max(hi, window_[i]);
    }
    swing_deg_ = 0.5 * static_cast<double>(hi - lo) * 360.0 / calib_.cpr;
}

void ActuatorTest::startRung(int i) {
    rung_ = i;
    rep_ = 1;
    repeat_ = false;
}

// The rep's trajectory, and a shaped move to where it starts.
void ActuatorTest::startRep(const hw::Sample& s) {
    traj_ = makeActTraj(c_, rungs_[static_cast<std::size_t>(rung_)], rep_);
    cur_ = RepResult{};
    cur_.rung = rung_;
    cur_.rep = rep_;
    cur_.repeat = repeat_;
    cur_.v = rungs_[static_cast<std::size_t>(rung_)].v;
    cur_.a = rungs_[static_cast<std::size_t>(rung_)].a;
    mover_.start(posAt(traj_->at(0).x), c_.v_travel, Side::None, s);
    phase_ = ActPhase::ToStart;
    t_phase_ = 0;
}

// Touch the nearer switch: travel at v_travel to `approach` before where the step count says it is (closing
// earlier also counts as found), then seek at v_seek up to touch_tol + 5 mm beyond it.
void ActuatorTest::startTouch(const hw::Sample& s) {
    const double upm = calib_.usteps_per_m;
    side_ = (s.pos_usteps - pos_r_) <= (pos_l_ - s.pos_usteps) ? Side::Right : Side::Left;
    const std::int64_t dir = side_ == Side::Right ? -1 : 1;           // toward the switch, in pos
    expected_ = side_ == Side::Right ? pos_r_ : pos_l_;
    const std::int64_t approachPos = expected_ - dir * std::llround(c_.approach * upm);
    if (dir * (approachPos - s.pos_usteps) > 0) {
        mover_.start(approachPos, c_.v_travel, side_, s, false);
        step_ = 0;
    } else {
        mover_.start(expected_ + dir * std::llround((c_.touch_tol + 0.005) * upm), c_.v_seek, side_, s);
        step_ = 1;
    }
    phase_ = ActPhase::Touch;
    t_phase_ = 0;
}

void ActuatorTest::finishRep(Verdict v) {
    cur_.verdict = v;
    results_.push_back(cur_);
}

// The switch was touched: audit, re-zero on it, then the next rep, a repeat, or back to the centre.
void ActuatorTest::afterTouch(hw::Hardware& hw, const hw::Sample& s, ActRecord& r) {
    const double upm = calib_.usteps_per_m;
    const std::int64_t err = mover_.switchPos() - expected_;
    last_side_ = side_;
    cur_.side = side_ == Side::Right ? 1 : 2;
    cur_.touch_err = err;
    r.event = static_cast<int>(ActEvent::Touched);
    if (std::fabs(static_cast<double>(err)) > c_.touch_tol * upm) {
        finishRep(Verdict::Fail);
        rung_verdict_[static_cast<std::size_t>(rung_)] = Verdict::Fail;
        failWith(hw, "%s switch found %.1f mm from where the step count puts it (tolerance %.0f mm): step loss or a "
                 "step-mode change", side_ == Side::Right ? "right" : "left", static_cast<double>(err) / upm * 1e3,
                 c_.touch_tol * 1e3);
        r.event = static_cast<int>(ActEvent::Failed);
        return;
    }
    pos_r_ += err;                                                    // re-zero on the switch
    pos_l_ += err;
    centre_ += err;
    calib_.pos_center = static_cast<double>(centre_);
    const double e = std::fabs(static_cast<double>(err));
    Verdict v = e <= c_.touch_pass ? Verdict::Pass : e >= c_.touch_fail ? Verdict::Fail
                                                    : (repeat_ ? Verdict::Fail : Verdict::Marginal);
    finishRep(v);
    if (v == Verdict::Marginal) {                                     // the same rep once more
        repeat_ = true;
        startRep(s);
        return;
    }
    repeat_ = false;
    const std::size_t g = static_cast<std::size_t>(rung_);
    if (v == Verdict::Fail) {
        rung_verdict_[g] = Verdict::Fail;
        for (std::size_t i = g + 1; i < rungs_.size(); ++i)          // the rest of this speed's ladder
            if (rungs_[i].speed_index == rungs_[g].speed_index && rung_verdict_[i] == Verdict::NotRun)
                rung_verdict_[i] = Verdict::Skipped;
    } else if (rep_ < c_.reps) {
        ++rep_;
        startRep(s);
        return;
    } else {
        rung_verdict_[g] = Verdict::Pass;
    }
    r.event = static_cast<int>(ActEvent::RungDone);
    mover_.start(centre_, c_.v_travel, Side::None, s);
    phase_ = ActPhase::Centre;
    t_phase_ = 0;
}

// BRAKE: after the trajectory, or at once on an abort, at the rung's acceleration until the Axis stands still.
void ActuatorTest::brakeTick(hw::Hardware& hw, const hw::Sample& s, ActRecord& r, bool quit) {
    const double ab = rung_ >= 0 ? rungs_[static_cast<std::size_t>(rung_)].a : c_.hold_a;
    const double a = r.v != 0.0 ? -std::copysign(std::min(ab, std::fabs(r.v) / c_.ts), r.v) : 0.0;
    r.a_cmd = a;
    r.axis_arg = core::axisAccelArg(a, calib_);
    hw.commandAccel(r.axis_arg);
    r.sent = true;
    still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
    ++brake_ticks_;
    if (still_ >= 25) {                          // 50 ms at standstill
        if (aborting_) {
            driver(hw, false);                   // reason_ was set when the RUN was aborted
            phase_ = quit ? ActPhase::Finished : ActPhase::Failed;
            r.event = static_cast<int>(quit ? ActEvent::Finished : ActEvent::Failed);
        } else if (quit) {
            finishRep(Verdict::Aborted);         // it ran, but there is no audit
            phase_ = ActPhase::Finishing;
            still_ = 0;
            t_phase_ = 0;
        } else {
            startTouch(s);
        }
    } else if (brake_ticks_ * c_.ts > 3.0) {     // never expected: the Axis does not stop
        hw.halt();
        failWith(hw, "brake: no standstill within 3 s (HALT)");
        r.event = static_cast<int>(ActEvent::Failed);
    }
}

ActRecord ActuatorTest::tick(hw::Hardware& hw, bool overrun, std::int64_t lateNs) {
    ActRecord r;
    r.overrun = overrun;
    const bool go = go_.exchange(false);
    const bool quit = quit_.load();
    const double ts = c_.ts;
    const double upm = calib_.usteps_per_m;
    t_phase_ += ts;
    overruns_ = overrun ? overruns_ + 1 : 0;

    const hw::Sample s = hw.read();
    if (!has_start_) {                         // x before HOME: from the start position
        calib_.pos_center = static_cast<double>(s.pos_usteps);
        has_start_ = true;
    }
    r.t_ns = s.t_ns; r.counts = s.counts; r.pos_usteps = s.pos_usteps; r.speed_toggles = s.speed_toggles;
    r.n_edges = s.n_edges; r.limit_left = s.limit_left; r.limit_right = s.limit_right;
    r.axis_accelerating = s.axis_accelerating;
    r.encoder_fault = s.encoder_fault || s.edges_overflow;
    r.x = core::xFromPos(s.pos_usteps, calib_);
    r.v = core::vFromAxisSpeed(s.speed_toggles, calib_);
    updateSwing(s.counts);
    r.swing_deg = swing_deg_;

    switch (phase_) {
        case ActPhase::WaitGo:
            if (quit) { phase_ = ActPhase::Finished; r.event = static_cast<int>(ActEvent::Finished); break; }
            if (go) {
                driver(hw, true);
                mover_.start(s.pos_usteps - std::llround(c_.seek_max * upm), c_.v_seek, Side::Right, s);
                step_ = 0;
                phase_ = ActPhase::Home;
                t_phase_ = 0;
                r.event = static_cast<int>(ActEvent::HomeStarted);
            }
            break;

        case ActPhase::Home:
        case ActPhase::ToStart:
        case ActPhase::Touch:
        case ActPhase::Centre: {
            const MoverRecord m = mover_.tick(hw, s, overrun, quit);
            r.a_cmd = m.a_cmd; r.axis_arg = m.axis_arg; r.sent = m.sent;
            if (mover_.state() == MoverState::Failed) {
                if (phase_ == ActPhase::Touch) finishRep(Verdict::Aborted);   // the rep ran, its audit did not
                if (quit && std::strcmp(mover_.reason(), "stop request") == 0) {
                    driver(hw, false);
                    phase_ = ActPhase::Finished;
                    r.event = static_cast<int>(ActEvent::Finished);
                } else if (phase_ == ActPhase::Touch && step_ == 1 && !mover_.found() &&
                           std::strcmp(mover_.reason(), "switch not found before the give-up position") == 0) {
                    failWith(hw, "%s switch not within %.0f mm of where the step count puts it: step loss or a "
                             "step-mode change", side_ == Side::Right ? "right" : "left", (c_.touch_tol + 0.005) * 1e3);
                    r.event = static_cast<int>(ActEvent::Failed);
                } else {
                    failWith(hw, "%s: %s", phase_ == ActPhase::Home ? "homing" : phase_ == ActPhase::Touch ? "touch"
                                           : "move", mover_.reason());
                    r.event = static_cast<int>(ActEvent::Failed);
                }
                break;
            }
            if (mover_.state() != MoverState::Done) break;
            if (phase_ == ActPhase::Home) {
                if (step_ == 0) {                                     // right switch found
                    pos_r_ = mover_.switchPos();
                    last_side_ = Side::Right;
                    r.event = static_cast<int>(ActEvent::SwitchFound);
                    mover_.start(pos_r_ + std::llround(c_.span_usteps * (1 + c_.span_tol) + 0.01 * upm), c_.v_seek,
                                 Side::Left, s);
                    step_ = 1;
                } else if (step_ == 1) {                              // left switch found: span check
                    pos_l_ = mover_.switchPos();
                    last_side_ = Side::Left;
                    r.event = static_cast<int>(ActEvent::SwitchFound);
                    const double span = static_cast<double>(pos_l_ - pos_r_);
                    if (std::fabs(span - c_.span_usteps) > c_.span_tol * c_.span_usteps) {
                        failWith(hw, "track span %.0f usteps (%.3f m at %.0f usteps/m), expected %.0f +- %.0f %%: the "
                                 "step mode is not the configured one", span, span / upm, upm, c_.span_usteps,
                                 c_.span_tol * 100);
                        r.event = static_cast<int>(ActEvent::Failed);
                        break;
                    }
                    centre_ = pos_r_ + (pos_l_ - pos_r_) / 2;
                    mover_.start(centre_, c_.v_travel, Side::None, s);
                    step_ = 2;
                } else {                                              // centred
                    calib_.pos_center = static_cast<double>(centre_);
                    phase_ = ActPhase::Ready;
                    t_phase_ = 0;
                    r.event = static_cast<int>(ActEvent::Homed);
                }
            } else if (phase_ == ActPhase::ToStart) {
                phase_ = ActPhase::Settle;
                t_phase_ = 0;
            } else if (phase_ == ActPhase::Touch) {
                if (step_ == 0 && !mover_.found()) {                  // at the approach point: seek
                    const std::int64_t dir = side_ == Side::Right ? -1 : 1;
                    mover_.start(expected_ + dir * std::llround((c_.touch_tol + 0.005) * upm), c_.v_seek, side_, s);
                    step_ = 1;
                } else {
                    afterTouch(hw, s, r);
                }
            } else {                                                  // CENTRE done
                phase_ = ActPhase::Ready;
                t_phase_ = 0;
                r.event = static_cast<int>(ActEvent::Ready);
            }
            break;
        }

        case ActPhase::Ready:
            r.a_cmd = hold(hw, r.v);
            r.axis_arg = core::axisAccelArg(r.a_cmd, calib_);
            r.sent = true;
            if (quit) { phase_ = ActPhase::Finishing; still_ = 0; t_phase_ = 0; break; }
            if (skip_.exchange(false)) {
                const int n = nextRung();
                if (n >= 0) {
                    const int si = rungs_[static_cast<std::size_t>(n)].speed_index;
                    for (std::size_t i = static_cast<std::size_t>(n); i < rungs_.size(); ++i)
                        if (rungs_[i].speed_index == si && rung_verdict_[i] == Verdict::NotRun) rung_verdict_[i] = Verdict::Skipped;
                    r.event = static_cast<int>(ActEvent::Skipped);
                }
            } else if (go) {
                const int n = nextRung();
                if (n < 0) { phase_ = ActPhase::Finishing; still_ = 0; t_phase_ = 0; break; }
                startRung(n);
                startRep(s);
                r.event = static_cast<int>(ActEvent::RepStarted);
            }
            break;

        case ActPhase::Settle: {
            r.a_cmd = hold(hw, r.v);
            r.axis_arg = core::axisAccelArg(r.a_cmd, calib_);
            r.sent = true;
            if (quit) { phase_ = ActPhase::Finishing; still_ = 0; t_phase_ = 0; break; }
            const double T0 = 2 * kPi / c_.w0;
            const bool calm = swing_deg_ < c_.settle_deg, late = t_phase_ > c_.settle_max;
            if (t_phase_ >= T0 && (calm || late)) {
                cur_.settle_timeout = !calm;
                cur_.swing_start_deg = swing_deg_;
                phase_ = ActPhase::Run;
                t_phase_ = 0;
                t_run_ = 0;
                aborting_ = false;
                r.event = static_cast<int>(calm ? ActEvent::RunStarted : ActEvent::SettleTimeout);
            }
            break;
        }

        case ActPhase::Run: {
            const ActRung& g = rungs_[static_cast<std::size_t>(rung_)];
            cur_.worst_late_ms = std::max(cur_.worst_late_ms, static_cast<double>(lateNs) * 1e-6);
            const core::ProfilePoint ref = traj_->at(t_run_);
            r.x_ref = ref.x; r.v_ref = ref.v; r.a_ref = ref.a;
            const char* why = nullptr;
            if (s.limit_left || s.limit_right) why = s.limit_left ? "unexpected limit switch (left)" : "unexpected limit switch (right)";
            else if (quit) why = "stop request";
            else if (overruns_ >= c_.overrun_streak) why = "loop overrun streak";
            else if (sent_prev_ && !s.axis_accelerating) why = "Axis not in ACCELERATE";
            else if (s.encoder_fault || s.edges_overflow) why = "encoder fault";
            else if (std::fabs(ref.x - r.x) > c_.track_tol) why = "tracking error";
            if (why) {
                std::snprintf(reason_, sizeof reason_, "rung %d (v %.2f m/s, a %.1f m/s^2) rep %d aborted: %s", rung_ + 1,
                              g.v, g.a, rep_, why);
                cur_.t_run = t_run_;
                finishRep(Verdict::Aborted);
                rung_verdict_[static_cast<std::size_t>(rung_)] = Verdict::Aborted;
                aborting_ = true;
                phase_ = ActPhase::Brake;
                still_ = brake_ticks_ = 0;
                t_phase_ = 0;
                r.event = static_cast<int>(ActEvent::RunEnded);
                brakeTick(hw, s, r, quit);       // an abort brakes at once
            } else {
                const double a = core::trackAccel(ref, r.x, r.v, c_.kp, c_.kv, c_.a_margin * g.a);
                r.a_cmd = a;
                r.axis_arg = core::axisAccelArg(a, calib_);
                hw.commandAccel(r.axis_arg);
                r.sent = true;
                cur_.peak_a_cmd = std::max(cur_.peak_a_cmd, std::fabs(a));
                cur_.peak_v = std::max(cur_.peak_v, std::fabs(r.v));
                cur_.max_track = std::max(cur_.max_track, std::fabs(ref.x - r.x));
                t_run_ += ts;
                if (t_run_ > traj_->duration()) {
                    cur_.t_run = t_run_;
                    phase_ = ActPhase::Brake;
                    still_ = brake_ticks_ = 0;
                    t_phase_ = 0;
                    r.event = static_cast<int>(ActEvent::RunEnded);
                }
            }
            break;
        }

        case ActPhase::Brake:
            brakeTick(hw, s, r, quit);
            break;

        case ActPhase::Finishing: {
            const double a = r.v != 0.0 ? -std::copysign(std::min(c_.hold_a, std::fabs(r.v) / ts), r.v) : 0.0;
            r.a_cmd = a;
            r.axis_arg = core::axisAccelArg(a, calib_);
            if (driver_on_) { hw.commandAccel(r.axis_arg); r.sent = true; }
            still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
            if (still_ >= 25 || t_phase_ > 2.0) {
                if (t_phase_ > 2.0 && still_ < 25) hw.halt();
                driver(hw, false);
                phase_ = ActPhase::Finished;
                r.event = static_cast<int>(ActEvent::Finished);
            }
            break;
        }

        default:
            driver(hw, false);
            break;
    }
    sent_prev_ = r.sent;
    r.driver_enabled = driver_on_;
    r.phase = static_cast<int>(phase_);
    r.rung = rung_;
    r.rep = rep_;
    return r;
}

}
