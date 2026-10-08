#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "core/swingfit.h"

namespace {

std::vector<std::vector<double>> readNumCsv(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<double>> rows;
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::vector<double> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(std::strtod(cell.c_str(), nullptr));
        rows.push_back(f);
    }
    return rows;
}

}

TEST(fit_offset_matches_python) {
    auto edges = readNumCsv("tests/golden/swing_edges.csv");
    auto fit = readNumCsv("tests/golden/swing_fit.csv");
    CHECK(edges.size() > 100);
    CHECK_EQ(fit.size(), 1u);
    std::vector<double> t, u;
    for (const auto& r : edges) { t.push_back(r[0]); u.push_back(r[1]); }
    const double w = fit[0][0], sigma = fit[0][1];
    auto f = core::fitOffset(t.data(), u.data(), t.size(), w, sigma);
    CHECK(f.ok);
    CHECK_NEAR(f.c, fit[0][2], 1e-8);
    CHECK_NEAR(f.se, fit[0][3], 1e-8 + 1e-6 * fit[0][3]);
    CHECK_NEAR(f.c, 123.37, 0.02);    // and it is close to the true equilibrium
    std::cout << "    fitOffset: c = " << f.c << " +- " << f.se << " counts from " << t.size() << " edges\n";
}

TEST(fit_offset_rejects_degenerate_input) {
    double t[3] = {0, 0.1, 0.2}, u[3] = {1, 2, 3};
    CHECK(!core::fitOffset(t, u, 3, 6.0, 0.0).ok);          // too few points
    double t2[10], u2[10];
    for (int i = 0; i < 10; ++i) { t2[i] = 0.0; u2[i] = 5.0; }  // all at one instant: singular
    CHECK(!core::fitOffset(t2, u2, 10, 6.0, 0.0).ok);
}
