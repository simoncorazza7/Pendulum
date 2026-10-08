#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "core/estimator.h"
#include "core/pid.h"

namespace {

std::vector<std::vector<double>> readNum(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<double>> rows;
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::vector<double> f;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) f.push_back(std::strtod(c.c_str(), nullptr));
        rows.push_back(f);
    }
    return rows;
}

const double kQ = 2 * 3.141592653589793 / 2400;   // rad per count

bool close(double a, double b) { return std::fabs(a - b) <= 1e-12 * std::max(1.0, std::fabs(b)); }

}

TEST(estimator_golden_matches_python) {
    auto rows = readNum("tests/golden/estimator_golden.csv");
    CHECK_EQ(rows.size(), 2000u);
    core::VelocityEstimator e1(core::VelocityEstimator::Kind::BdLp1, 0.002, 30.0);
    core::VelocityEstimator e2(core::VelocityEstimator::Kind::BdLp2, 0.002, 30.0);
    core::VelocityEstimator e3(core::VelocityEstimator::Kind::BdLp2, 0.002, 50.0);
    int bad = 0;
    double worst = 0;
    for (const auto& r : rows) {
        const auto c = static_cast<std::int64_t>(r[1]);
        const double got[3] = {e1.update(c) * kQ, e2.update(c) * kQ, e3.update(c) * kQ};
        for (int i = 0; i < 3; ++i) {
            worst = std::max(worst, std::fabs(got[i] - r[2 + i]) / std::max(1.0, std::fabs(r[2 + i])));
            if (!close(got[i], r[2 + i])) ++bad;
        }
    }
    std::cout << "    golden: 2000 samples x 3 estimators, worst relative difference " << worst << "\n";
    CHECK_EQ(bad, 0);
}

TEST(pid_external_velocity_matches_python) {
    auto rows = readNum("tests/golden/pid_ext_golden.csv");
    CHECK_EQ(rows.size(), 2000u);
    const auto g = core::designGains(2 * 3.141592653589793, 0.1, 4 * 3.141592653589793, 0.7, 3.141592653589793);
    core::PidParams p;
    p.kp = g.kp; p.ki = g.ki; p.kd = g.kd; p.a_max = 5.0; p.v_max = 0.8; p.i_max = 1.0; p.i_delay = 0.15;
    p.theta_ref = 0.001;
    core::PidState s;
    int bad = 0, sat = 0;
    for (const auto& r : rows) {
        const double vel = r[3];
        auto res = core::pidStep(p, s, r[1], r[2], 0.002, &vel);
        s = res.state;
        if (!close(res.a_cmd, r[4]) || !close(res.u, r[5]) || !close(s.integral, r[6]) ||
            (res.saturated ? 1 : 0) != static_cast<int>(r[7]))
            ++bad;
        sat += res.saturated;
    }
    CHECK_EQ(bad, 0);
    CHECK(sat > 10);
}

TEST(estimator_constant_rate_and_rest) {
    for (auto kind : {core::VelocityEstimator::Kind::BdLp1, core::VelocityEstimator::Kind::BdLp2}) {
        core::VelocityEstimator e(kind, 0.002, 30.0);
        double v = 0;
        for (int k = 0; k < 500; ++k) v = e.update(3 * k);   // 3 counts per sample = 1500 counts/s
        CHECK_NEAR(v, 1500.0, 1e-6);
        for (int k = 0; k < 500; ++k) v = e.update(1497);    // at rest
        CHECK_NEAR(v, 0.0, 1e-6);
        e.reset();
        CHECK_EQ(e.update(123456), 0.0);                    // first sample after reset: no spike
    }
}
