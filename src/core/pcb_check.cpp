#include "core/pcb_check.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace core {
namespace {

std::string fmt(const char* f, double a) {
    char b[160];
    std::snprintf(b, sizeof b, f, a);
    return b;
}

// BCM number on each physical header pin (-1 = power / ground).
const int kHeaderBcm[41] = {-1,                                   // (unused index 0)
    -1, -1, 2, -1, 3, -1, 4, 14, -1, 15, 17, 18, 27, -1, 22, 23, -1, 24, 10, -1,
    9, 25, 11, 8, -1, 7, 0, 1, 5, -1, 6, 12, 13, -1, 19, 16, 26, 20, -1, 21};

}

const char* levelName(Level l) {
    switch (l) {
        case Level::Pass: return "PASS";
        case Level::Info: return "INFO";
        case Level::Warn: return "WARN";
        case Level::Fail: return "FAIL";
        case Level::Skip: return "SKIP";
    }
    return "?";
}

// pcb/PCB.kicad_pcb (2026-10-06): J1 = Pi header, A1 = Pololu A4988 carrier, J3 = limit switches, J4 = encoder.
const std::vector<BoardLine>& boardLines() {
    static const std::vector<BoardLine> lines = {
        {0, 27, "not connected (ID_SD)", LineLoad::None, false},
        {1, 28, "not connected (ID_SC)", LineLoad::None, false},
        {2, 3, "RIGHT limit switch (J3.2); R4 10k + the Pi's 1.8k pull-up", LineLoad::PullUp, false},
        {3, 5, "not connected; the Pi's 1.8k pull-up (SCL1)", LineLoad::PullUp, false},
        {4, 7, "not connected", LineLoad::None, false},
        {5, 29, "not connected", LineLoad::None, false},
        {6, 31, "not connected", LineLoad::None, false},
        {7, 26, "not connected", LineLoad::None, false},
        {8, 24, "not connected", LineLoad::None, false},
        {9, 21, "LEFT limit switch (J3.3); R3 10k pull-up", LineLoad::PullUp, false},
        {10, 19, "encoder A (J4.2); R1 4.7k pull-up", LineLoad::PullUp, false},
        {11, 23, "not connected", LineLoad::None, false},
        {12, 32, "not connected", LineLoad::None, false},
        {13, 33, "not connected", LineLoad::None, false},
        {14, 8, "not connected (UART TX)", LineLoad::None, false},
        {15, 10, "not connected (UART RX)", LineLoad::None, false},
        {16, 36, "not connected", LineLoad::None, false},
        {17, 11, "not connected", LineLoad::None, false},
        {18, 12, "not connected", LineLoad::None, false},
        {19, 35, "encoder B (J4.3); R2 4.7k pull-up", LineLoad::PullUp, false},
        {20, 38, "MS1 -> A4988 pin 10 (100k pull-down inside the A4988)", LineLoad::WeakPullDown, true},
        {21, 40, "MS2 -> A4988 pin 11 (50k pull-down inside the A4988)", LineLoad::PullDown50k, true},
        {22, 15, "DIR -> A4988 pin 16", LineLoad::None, true},
        {23, 16, "not connected", LineLoad::None, false},
        {24, 18, "~ENABLE -> A4988 pin 9 (no pull-up on the board)", LineLoad::None, true},
        {25, 22, "not connected", LineLoad::None, false},
        {26, 37, "not connected", LineLoad::None, false},
        {27, 13, "STEP -> A4988 pin 15", LineLoad::None, true},
    };
    return lines;
}

const BoardLine* boardLine(int bcm) {
    for (const BoardLine& l : boardLines())
        if (l.bcm == bcm) return &l;
    return nullptr;
}

std::string headerPinName(int header) {
    if (header < 1 || header > 40) return "?";
    if (header == 1 || header == 17) return "3V3";
    if (header == 2 || header == 4) return "5V";
    const int b = kHeaderBcm[header];
    return b < 0 ? "GND" : "GPIO" + std::to_string(b);
}

std::string headerNeighbours(int header) {
    // Two rows: odd pins on one, even on the other, 2.54 mm apart both ways.
    std::vector<int> n;
    if (header - 2 >= 1) n.push_back(header - 2);
    if (header + 2 <= 40) n.push_back(header + 2);
    n.push_back(header % 2 ? header + 1 : header - 1);
    std::sort(n.begin(), n.end());
    std::string s;
    for (int p : n) s += (s.empty() ? "" : ", ") + ("pin " + std::to_string(p) + " (" + headerPinName(p) + ")");
    return s;
}

Judgement judgePull(const BoardLine& l, int up, int dn) {
    const std::string near = " Header pin " + std::to_string(l.header) + "; neighbours: " + headerNeighbours(l.header) + ".";
    if (up == 0 && dn == 1) return {Level::Fail, "reads the opposite of each pull: something on the board drives it." + near};
    switch (l.load) {
        case LineLoad::None:
            if (up == 1 && dn == 0)
                return {Level::Pass, l.output ? "follows the internal pull: A4988 input, nothing else on it (as designed)"
                                              : "follows the internal pull: nothing on it (as designed)"};
            if (up == 1) return {Level::Fail, "held HIGH against the pull-down: a bridge to 3V3/5V or to a pulled-up line." + near};
            return {Level::Fail, "held LOW against the pull-up: a bridge to GND or to a line held low." + near};
        case LineLoad::PullUp:
            if (up == 1 && dn == 1) return {Level::Pass, "pulled up (as designed)"};
            if (up == 1)
                return {Level::Fail, "follows the internal pull-down: its pull-up resistor is missing or not soldered "
                                     "(check the joint)." + near};
            return {Level::Fail, "held LOW: a short to GND, or the switch/encoder is plugged in (stage C needs J3 and J4 "
                                 "empty)." + near};
        case LineLoad::WeakPullDown:
            if (up == 1 && dn == 0) return {Level::Pass, "follows the internal pull against the A4988's 100k pull-down (as designed)"};
            if (up == 0 && dn == 0)
                return {Level::Warn, "reads 0 even with the internal pull-up: a pull-down stronger than the A4988's 100k, "
                                     "or a leak to GND. The drive test decides." + near};
            return {Level::Fail, "held HIGH against the pull-down: a bridge to 3V3/5V or a pulled-up line." + near};
        case LineLoad::PullDown50k:
            if (dn == 0)
                return {Level::Pass, std::string("pulled down by the A4988's 50k (as designed; the pull-up reading, ") +
                                         (up ? "1" : "0") + ", is undefined at ~1.65 V)"};
            return {Level::Fail, "held HIGH against the pull-down: a bridge to 3V3/5V or a pulled-up line." + near};
    }
    return {Level::Fail, "?"};
}

std::vector<Bridge> findBridges(const std::vector<int>& bcm, const std::vector<int>& baseUp,
                                const std::vector<int>& baseDown, const std::vector<std::vector<int>>& high,
                                const std::vector<std::vector<int>>& low) {
    std::vector<Bridge> out;
    auto add = [&](int a, int b, bool h) {
        const int lo = std::min(a, b), hi = std::max(a, b);
        for (Bridge& x : out)
            if (x.a == lo && x.b == hi) { (h ? x.high : x.low) = true; return; }
        out.push_back({lo, hi, h, !h});
    };
    const std::size_t n = bcm.size();
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j) continue;
            const BoardLine* lj = boardLine(bcm[j]);
            const bool lowWitness = baseUp[j] == 1 && !(lj && (lj->load == LineLoad::WeakPullDown || lj->load == LineLoad::PullDown50k));
            if (i < high.size() && j < high[i].size() && high[i][j] == 1 && baseDown[j] == 0) add(bcm[i], bcm[j], true);
            if (i < low.size() && j < low[i].size() && low[i][j] == 0 && lowWitness) add(bcm[i], bcm[j], false);
        }
    std::sort(out.begin(), out.end(), [](const Bridge& x, const Bridge& y) { return x.a != y.a ? x.a < y.a : x.b < y.b; });
    return out;
}

EncoderTurn analyzeTurn(std::int64_t count0, const std::vector<EdgeSample>& e) {
    EncoderTurn t;
    t.edges = static_cast<long>(e.size());
    std::int64_t prev = count0;
    int lastSign = 0;
    double sum[4] = {};
    // Edge indices; consecutive entries are one single step apart, all in the same direction. Four steps (five
    // points) are one quadrature cycle. A run starts at an edge (a point), so the step that led to it (from count0,
    // or a reversal) is never part of a cycle.
    std::vector<std::size_t> run;
    auto flushCycle = [&]() {
        while (run.size() >= 5) {
            const std::int64_t t0 = e[run[0]].ts_ns, total = e[run[4]].ts_ns - t0;
            if (total > 0 && total < 500000000LL) {
                double part[4];
                int cls[4];
                bool ok = true;
                for (int k = 0; k < 4; ++k) {
                    const std::int64_t dt = e[run[k + 1]].ts_ns - e[run[k]].ts_ns;
                    if (dt <= 0) { ok = false; break; }
                    part[k] = static_cast<double>(dt) / static_cast<double>(total);
                    const std::int64_t lower = std::min(e[run[k]].count, e[run[k + 1]].count);
                    cls[k] = static_cast<int>(((lower % 4) + 4) % 4);
                }
                if (ok) {
                    for (int k = 0; k < 4; ++k) sum[cls[k]] += part[k];
                    ++t.cycles;
                }
            }
            run.erase(run.begin(), run.begin() + 4);
        }
    };
    for (std::size_t i = 0; i < e.size(); ++i) {
        const std::int64_t d = e[i].count - prev;
        if (d == 0 || std::llabs(d) > 1) {
            if (std::llabs(d) > 1) ++t.jumps;
            run.clear();
            prev = e[i].count;
            continue;
        }
        const std::int64_t lower = std::min(prev, e[i].count);
        (lower % 2 == 0 ? t.chanEven : t.chanOdd)++;
        const int s = d > 0 ? 1 : -1;
        if (lastSign != 0 && s != lastSign) {
            ++t.reversals;
            run.clear();
        }
        lastSign = s;
        run.push_back(i);
        flushCycle();
        prev = e[i].count;
    }
    t.net = static_cast<long>(prev - count0);
    for (int k = 0; k < 4; ++k) t.quarter[k] = t.cycles ? sum[k] / static_cast<double>(t.cycles) : 0.0;
    return t;
}

Judgement judgeTurn(const EncoderTurn& t, long expect, long tol) {
    if (t.edges == 0)
        return {Level::Fail, "no encoder edges: not powered (J4.1 5 V / J4.4 GND), signal wires not on J4.2/J4.3, or the "
                             "shaft didn't turn"};
    if (t.edges > 40 && (t.chanEven == 0 || t.chanOdd == 0))
        return {Level::Fail, "only one channel changes (" + std::to_string(t.chanEven) + " / " + std::to_string(t.chanOdd) +
                                 " edges): channel A or B is dead (J4.2/J4.3, R1/R2, the encoder cable)"};
    std::string extra;
    if (t.jumps) extra = "; " + std::to_string(t.jumps) + " count jumps (missed edges)";
    const std::string got = "net " + std::to_string(t.net) + " counts (expected " + std::to_string(expect) + " +- " +
                            std::to_string(tol) + "), " + std::to_string(t.edges) + " edges, channels " +
                            std::to_string(t.chanEven) + " / " + std::to_string(t.chanOdd);
    if (std::labs(t.net - expect) <= tol) return {t.jumps ? Level::Warn : Level::Pass, got + extra};
    if (std::labs(t.net + expect) <= tol)
        return {Level::Fail, got + ": the encoder counts the OTHER way, i.e. A and B are swapped (J4.2/J4.3 or the encoder "
                                   "cable). With calib.theta_sign = +1 the controller would then push the pendulum over: "
                                   "swap the two signal wires before any balancing"};
    return {Level::Fail, got + extra + ": not one clean revolution. Repeat slowly and let the pendulum come to rest "
                                       "before pressing Enter; if it repeats, counts are being lost"};
}

Judgement judgeQuadrature(const EncoderTurn& a, const EncoderTurn& b) {
    const long n = a.cycles + b.cycles;
    if (n < 20) return {Level::Info, "too little steady rotation to judge the quadrature timing (" + std::to_string(n) + " cycles)"};
    double q[4], worst = 0;
    for (int k = 0; k < 4; ++k) {
        q[k] = (a.quarter[k] * static_cast<double>(a.cycles) + b.quarter[k] * static_cast<double>(b.cycles)) / static_cast<double>(n);
        worst = std::max(worst, std::fabs(q[k] - 0.25));
    }
    char s[200];
    std::snprintf(s, sizeof s, "quarters of a cycle %.2f / %.2f / %.2f / %.2f (ideal 0.25 each), %ld cycles", q[0], q[1],
                  q[2], q[3], n);
    if (worst <= 0.06) return {Level::Pass, s};
    return {Level::Warn, std::string(s) + ": the A/B duty cycle or phase is off (encoder alignment, not the PCB). The "
                                          "decoder copes, but edges this close are where flickers show up"};
}

std::vector<Judgement> judgeStepModes(const std::vector<StepTrial>& tr, double tol) {
    std::vector<Judgement> out;
    if (tr.empty()) return out;
    const StepTrial& ref = tr[0];
    const double refDist = static_cast<double>(ref.pulses) * ref.usteps_per_pulse;
    for (std::size_t i = 0; i < tr.size(); ++i) {
        const StepTrial& t = tr[i];
        const std::string name = std::string(t.name) + " (" + std::to_string(t.pulses) + " pulses)";
        if (t.returned < 0 || ref.returned < 0) {
            out.push_back({Level::Fail, name + ": the switch was not reached on the way back"});
            continue;
        }
        const double expected = static_cast<double>(t.pulses) * t.usteps_per_pulse;
        const double dist = static_cast<double>(t.returned - ref.returned) + refDist;
        if (i == 0) {
            out.push_back({Level::Pass, name + ": reference, " + std::to_string(t.returned) + " eighth-steps back to the switch"});
            continue;
        }
        const double ratio = dist / expected;
        if (std::fabs(ratio - 1.0) <= tol) {
            out.push_back({Level::Pass, name + fmt(": moved %.0f eighth-steps", dist) + fmt(" of %.0f expected", expected) +
                                            fmt(" (%.3f)", ratio)});
            continue;
        }
        // Which mode did it really step in?
        const double eff = ratio * t.usteps_per_pulse;
        const int modes[4] = {1, 2, 4, 8};
        const char* names[4] = {"1/8", "1/4", "1/2", "full"};
        int best = 0;
        for (int k = 1; k < 4; ++k)
            if (std::fabs(std::log(eff / modes[k])) < std::fabs(std::log(eff / modes[best]))) best = k;
        out.push_back({Level::Fail, name + fmt(": moved %.0f eighth-steps", dist) + fmt(" of %.0f expected", expected) +
                                        fmt(" (%.2fx): the driver stepped like ", ratio) + names[best] +
                                        ". Check MS1/MS2 (GPIO20/21 -> A4988 pins 10/11) for an open joint, a bridge or a "
                                        "swap"});
    }
    return out;
}

Judgement judgeThrottled(unsigned v, const char* when) {
    char s[160];
    std::snprintf(s, sizeof s, "%s: get_throttled = 0x%x", when, v);
    if (v & 0x10001u)
        return {Level::Fail, std::string(s) + ((v & 1u) ? ": UNDER-VOLTAGE now" : ": under-voltage has occurred since boot") +
                                 ". The 5 V supply sags below ~4.63 V under load: check U1's output and the 5 V path"};
    if (v & 0xC000Cu) return {Level::Warn, std::string(s) + ": throttling / temperature limit (not the PCB's supply)"};
    if (v) return {Level::Warn, std::string(s) + ": frequency capped"};
    return {Level::Pass, std::string(s) + ": no under-voltage"};
}

Judgement judgeRange(const std::string& what, double v, double lo, double hi, const char* unit) {
    char s[200];
    std::snprintf(s, sizeof s, "%s %.3g %s (expected %.3g-%.3g)", what.c_str(), v, unit, lo, hi);
    return {v >= lo && v <= hi ? Level::Pass : Level::Fail, s};
}

}
