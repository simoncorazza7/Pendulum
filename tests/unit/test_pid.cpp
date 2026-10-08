#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "core/pid.h"

using core::PidParams;
using core::PidState;
using core::pidStep;

namespace {

std::vector<std::vector<std::string>> readCsv(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<std::string>> rows;
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        rows.push_back(f);
    }
    return rows;
}

double num(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

bool close(double a, double b, double rel = 1e-12, double abs_ = 1e-12) {
    return std::fabs(a - b) <= abs_ + rel * std::max(std::fabs(a), std::fabs(b));
}

PidParams basic() {
    PidParams p;
    p.kp = 50; p.ki = 100; p.kd = 5; p.a_max = 5; p.v_max = 0.8;
    return p;
}

}

// ---- golden: C++ must reproduce the Python reference (py/make_golden.py) ----

TEST(pid_golden_matches_python) {
    auto cases = readCsv("tests/golden/pid_cases.csv");
    auto rows = readCsv("tests/golden/pid_golden.csv");
    CHECK(cases.size() == 3);
    CHECK(rows.size() > 5000);
    std::map<std::string, PidParams> params;
    for (const auto& c : cases) {
        PidParams p;
        p.kp = num(c[1]); p.ki = num(c[2]); p.kd = num(c[3]); p.a_max = num(c[4]); p.v_max = num(c[5]);
        p.v_taper = num(c[6]); p.i_max = num(c[7]); p.i_delay = num(c[8]); p.theta_ref = num(c[9]);
        p.vel_fc = num(c[10]);
        CHECK_EQ(core::validate(p), std::string());
        params[c[0]] = p;
    }
    std::map<std::string, PidState> state;
    int mismatches = 0, saturatedRows = 0;
    double worst = 0;
    for (const auto& r : rows) {
        const PidParams& p = params.at(r[0]);
        auto res = pidStep(p, state[r[0]], num(r[2]), num(r[3]), num(r[4]));
        state[r[0]] = res.state;
        const double exp[4] = {num(r[5]), num(r[6]), num(r[7]), num(r[8])};
        const double got[4] = {res.a_cmd, res.u, res.state.integral, res.state.vel_f};
        for (int i = 0; i < 4; ++i) {
            worst = std::max(worst, std::fabs(got[i] - exp[i]) / std::max(1.0, std::fabs(exp[i])));
            if (!close(got[i], exp[i])) ++mismatches;
        }
        if ((res.saturated ? 1 : 0) != std::atoi(r[9].c_str())) ++mismatches;
        saturatedRows += res.saturated;
    }
    std::cout << "    golden: " << rows.size() << " samples, worst relative difference " << worst
              << ", saturated in " << saturatedRows << "\n";
    CHECK_EQ(mismatches, 0);
    CHECK(saturatedRows > 100);   // the cases really exercise saturation
}

TEST(design_gains_match_python) {
    auto rows = readCsv("tests/golden/design_gains.csv");
    CHECK(rows.size() == 3);
    for (const auto& r : rows) {
        auto gn = core::designGains(num(r[0]), num(r[1]), num(r[2]), num(r[3]), num(r[4]));
        CHECK(close(gn.kp, num(r[5])));
        CHECK(close(gn.ki, num(r[6])));
        CHECK(close(gn.kd, num(r[7])));
    }
}

// ---- properties ----

TEST(pid_zero_error_zero_output) {
    PidState s;
    for (int k = 0; k < 10; ++k) {
        auto r = pidStep(basic(), s, 0.0, 0.0, 0.002);
        CHECK_EQ(r.a_cmd, 0.0);
        s = r.state;
    }
}

TEST(pid_sign_moves_cart_under_pendulum) {
    auto r = pidStep(basic(), PidState{}, 0.01, 0.0, 0.002);   // leaning toward +x
    CHECK(r.a_cmd > 0);                                         // accelerate toward +x
    CHECK_NEAR(r.a_cmd, 50 * 0.01, 1e-12);                      // first sample: P only
}

TEST(pid_saturates_at_a_max) {
    auto r = pidStep(basic(), PidState{}, 1.0, 0.0, 0.002);
    CHECK_EQ(r.a_cmd, 5.0);
    CHECK(r.saturated);
    r = pidStep(basic(), PidState{}, -1.0, 0.0, 0.002);
    CHECK_EQ(r.a_cmd, -5.0);
}

TEST(pid_v_max_taper_removes_authority_toward_the_limit) {
    PidParams p = basic();   // taper over 0.25 * 0.8 = 0.2 m/s
    CHECK_EQ(pidStep(p, PidState{}, 1.0, 0.8, 0.002).a_cmd, 0.0);       // at +v_max: no more +a
    CHECK_NEAR(pidStep(p, PidState{}, 1.0, 0.7, 0.002).a_cmd, 2.5, 1e-12); // halfway into the taper
    CHECK_EQ(pidStep(p, PidState{}, -1.0, 0.8, 0.002).a_cmd, -5.0);     // braking still allowed
}

TEST(pid_integrator_holds_while_saturated_outward) {
    PidState s;
    for (int k = 0; k < 100; ++k) s = pidStep(basic(), s, 1.0, 0.0, 0.002).state;   // saturated, e > 0
    CHECK_EQ(s.integral, 0.0);
    s = PidState{};
    for (int k = 0; k < 100; ++k) s = pidStep(basic(), s, 0.01, 0.0, 0.002).state;  // not saturated
    CHECK_NEAR(s.integral, 100 * 0.01 * 0.002, 1e-12);
}

TEST(pid_integrator_enable_delay_and_clamp) {
    PidParams p = basic();
    p.i_delay = 0.1;
    p.i_max = 0.05;   // |ki * integral| <= 0.05 m/s^2 -> |integral| <= 5e-4
    PidState s;
    for (int k = 0; k < 50; ++k) s = pidStep(p, s, 0.01, 0.0, 0.002).state;   // 0.1 s: still disabled
    CHECK_EQ(s.integral, 0.0);
    for (int k = 0; k < 1000; ++k) s = pidStep(p, s, 0.01, 0.0, 0.002).state;
    CHECK_NEAR(s.integral, 5e-4, 1e-15);
}

TEST(pid_derivative_on_measurement_not_reference) {
    PidParams p = basic();
    auto r1 = pidStep(p, PidState{}, 0.0, 0.0, 0.002);
    p.theta_ref = 0.02;                        // a trim change must not kick the D term
    auto r2 = pidStep(p, r1.state, 0.0, 0.0, 0.002);
    CHECK_EQ(r2.vel_raw, 0.0);
    CHECK_NEAR(r2.a_cmd, -50 * 0.02, 1e-12);
}

TEST(pid_validate_rejects_bad_params) {
    CHECK_EQ(core::validate(basic()), std::string());
    PidParams p = basic(); p.kp = 0;      CHECK(!core::validate(p).empty());
    p = basic(); p.v_taper = 0;           CHECK(!core::validate(p).empty());
    p = basic(); p.vel_fc = -1;           CHECK(!core::validate(p).empty());
    p = basic(); p.a_max = 0;             CHECK(!core::validate(p).empty());
}

TEST(delay_line_delays_by_n_samples) {
    core::DelayLine d0(0), d2(2);
    CHECK_EQ(d0.push(1.5), 1.5);
    CHECK_EQ(d2.push(1.0), 0.0);
    CHECK_EQ(d2.push(2.0), 0.0);
    CHECK_EQ(d2.push(3.0), 1.0);
    CHECK_EQ(d2.push(4.0), 2.0);
}
