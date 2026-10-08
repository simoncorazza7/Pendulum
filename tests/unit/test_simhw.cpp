#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "core/units.h"
#include "hw/sim_hw.h"

namespace {

struct Table {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    std::size_t col(const std::string& n) const {
        return static_cast<std::size_t>(std::find(header.begin(), header.end(), n) - header.begin());
    }
};

Table read(const std::string& path) {
    Table t;
    std::ifstream in(path);
    std::string line;
    bool first = true;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) f.push_back(c);
        if (first) { t.header = f; first = false; } else t.rows.push_back(f);
    }
    return t;
}

double D(const std::string& s) { return std::strtod(s.c_str(), nullptr); }
long long L(const std::string& s) { return std::atoll(s.c_str()); }
const double kPi = 3.141592653589793;

}

TEST(simhw_golden_matches_python) {
    Table P = read("tests/golden/simhw_params.csv");
    Table G = read("tests/golden/simhw_golden.csv");
    Table E = read("tests/golden/simhw_edges.csv");
    CHECK_EQ(P.rows.size(), 3u);
    std::map<std::string, std::vector<std::pair<long long, long long>>> edgesByKey;
    for (const auto& r : E.rows) edgesByKey[r[0] + ":" + r[1]].push_back({L(r[3]), L(r[4])});

    int intMismatch = 0, edgeMismatch = 0, samples = 0;
    double worstFloat = 0;
    for (const auto& pr : P.rows) {
        hw::SimHwParams p;
        p.w0 = D(pr[P.col("w0")]); p.bJ = D(pr[P.col("bJ")]); p.coulomb = D(pr[P.col("coulomb")]);
        p.truth.c_bottom = D(pr[P.col("c_bottom")]);
        p.truth.theta_sign = static_cast<int>(L(pr[P.col("theta_sign")]));
        p.truth.x_sign = static_cast<int>(L(pr[P.col("x_sign")]));
        hw::SimHw sim(p, D(pr[P.col("theta0")]));
        for (const auto& r : G.rows) {
            if (r[0] != pr[0]) continue;
            ++samples;
            const hw::Sample s = sim.read();
            const long long ints[] = {s.t_ns, s.counts, s.n_edges, s.edges_overflow, s.pos_usteps, s.speed_toggles,
                                      s.limit_right, s.limit_left, s.axis_accelerating, s.driver_enabled};
            const char* names[] = {"t_ns", "counts", "n_edges", "overflow", "pos", "speed", "lim_r", "lim_l",
                                   "axis_acc", "driver"};
            for (int i = 0; i < 10; ++i)
                if (ints[i] != L(r[G.col(names[i])])) ++intMismatch;
            const double fl[] = {sim.theta(), sim.thetadot(), sim.x(), sim.v()};
            const char* fn[] = {"theta", "thetadot", "x", "v"};
            for (int i = 0; i < 4; ++i)
                worstFloat = std::max(worstFloat, std::fabs(fl[i] - D(r[G.col(fn[i])])));
            const auto& want = edgesByKey[r[0] + ":" + r[1]];
            if (static_cast<int>(want.size()) != s.n_edges) ++edgeMismatch;
            else
                for (int i = 0; i < s.n_edges; ++i)
                    if (s.edges[static_cast<std::size_t>(i)].ts_ns != want[static_cast<std::size_t>(i)].first ||
                        s.edges[static_cast<std::size_t>(i)].count_after != want[static_cast<std::size_t>(i)].second)
                        ++edgeMismatch;
            const long long en = L(r[G.col("enable")]);
            if (en >= 0) sim.enableDriver(en == 1);
            if (L(r[G.col("halt")])) sim.halt();
            if (L(r[G.col("cmd_valid")])) sim.commandAccel(static_cast<int>(L(r[G.col("cmd_arg")])));
            sim.advance(0.002);
        }
    }
    std::cout << "    golden: " << samples << " SimHw samples, worst state difference " << worstFloat
              << ", integer mismatches " << intMismatch << ", edge mismatches " << edgeMismatch << "\n";
    CHECK_EQ(samples, 2900);
    CHECK(worstFloat < 1e-9);
    // Bit-identical on the same libm; another libm (the Pi's glibc) may flip a rounding at a boundary.
    CHECK(intMismatch <= 3);
    CHECK(edgeMismatch <= 3);
}

TEST(simhw_units_chain_recovers_truth) {
    for (int sign : {+1, -1}) {
        hw::SimHwParams p;
        p.truth.theta_sign = sign;
        p.truth.x_sign = sign;
        p.truth.c_bottom = 37.6;
        hw::SimHw sim(p, kPi + 0.2);
        sim.enableDriver(true);
        core::Calib rt = p.truth;
        rt.c_bottom = sim.trueCBottomRelative();   // what the start-up fit would find
        double worstTheta = 0, worstX = 0;
        for (int k = 0; k < 1000; ++k) {
            sim.commandAccel(core::axisAccelArg(0.8 * std::sin(0.01 * k), rt));
            sim.advance(0.002);
            const hw::Sample s = sim.read();
            worstTheta = std::max(worstTheta, std::fabs(core::wrap(core::thetaFromCounts(s.counts, rt) - sim.theta())));
            worstX = std::max(worstX, std::fabs(core::xFromPos(s.pos_usteps, rt) - sim.x()));
        }
        CHECK(worstTheta <= 0.5 * 2 * kPi / 2400 + 1e-9);      // within half a count (centred counts)
        CHECK(worstX <= 0.5 / rt.usteps_per_m + 1e-9);          // within half a microstep
    }
}

TEST(simhw_hanging_period_matches_w0) {
    hw::SimHwParams p;
    p.bJ = 0;
    hw::SimHw sim(p, kPi + 0.02);   // small swing, no friction, driver off (cart at rest)
    std::vector<double> upCross;
    double prev = sim.theta() - kPi;
    for (int k = 0; k < 5000; ++k) {
        sim.advance(0.001);
        const double cur = sim.theta() - kPi;
        if (prev < 0 && cur >= 0) upCross.push_back(sim.time() - 0.001 * cur / (cur - prev));
        prev = cur;
    }
    CHECK(upCross.size() >= 4);
    const double T = (upCross.back() - upCross.front()) / static_cast<double>(upCross.size() - 1);
    CHECK_NEAR(T, 2 * kPi / p.w0 * (1 + 0.02 * 0.02 / 16), 2e-4);   // with the amplitude correction
}

TEST(simhw_command_reaches_cart_after_delay_and_respects_cap) {
    hw::SimHwParams p;
    hw::SimHw sim(p, kPi);
    sim.enableDriver(true);
    core::Calib rt = p.truth;
    sim.commandAccel(core::axisAccelArg(10.0, rt));    // above the cap A = 4.5
    sim.advance(0.002);
    const double v1 = sim.v();                         // 1.5 ms of acceleration (0.5 ms delay)
    CHECK(v1 > 0 && v1 < 4.5 * 0.0015 + 1e-9);
    CHECK_NEAR(v1, 4.5 * 0.0015, 1e-4);
    const hw::Sample s = sim.read();
    CHECK(s.axis_accelerating);
    CHECK(s.speed_toggles < 0);                        // internal sign: +x speed is negative curSpeed
    sim.halt();
    for (int k = 0; k < 100 && !sim.isStopped(); ++k) sim.advance(0.002);
    CHECK(sim.isStopped());
    CHECK(!sim.read().axis_accelerating);
}
