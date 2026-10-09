#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "core/supervisor.h"
#include "core/units.h"

using namespace core;

namespace {

struct Csv {
    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
    int col(const std::string& name) const {
        auto it = std::find(header.begin(), header.end(), name);
        return it == header.end() ? -1 : static_cast<int>(it - header.begin());
    }
};

Csv readCsv(const std::string& path) {
    Csv c;
    std::ifstream in(path);
    std::string line;
    bool first = true;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        if (first) { c.header = f; first = false; } else c.rows.push_back(f);
    }
    return c;
}

double num(const std::string& s) { return std::strtod(s.c_str(), nullptr); }
bool close(double a, double b) { return std::fabs(a - b) <= 1e-12 + 1e-12 * std::max(std::fabs(a), std::fabs(b)); }

SupervisorParams paramsFrom(const Csv& c, const std::vector<std::string>& r) {
    SupervisorParams p;
    auto g = [&](const char* n) { return num(r[static_cast<std::size_t>(c.col(n))]); };
    p.theta_engage = g("theta_engage"); p.omega_engage = g("omega_engage");
    p.engage_samples = static_cast<int>(g("engage_samples")); p.theta_abort = g("theta_abort");
    p.x_min = g("x_min"); p.x_max = g("x_max"); p.a_brake = g("a_brake"); p.t_react = g("t_react");
    p.brake_margin = g("brake_margin"); p.axis_a = g("axis_a"); p.axis_v = g("axis_v");
    p.overrun_streak = static_cast<int>(g("overrun_streak")); p.max_run_time = g("max_run_time");
    p.v_stopped = g("v_stopped"); p.stop_timeout = g("stop_timeout"); p.hang_tol = g("hang_tol");
    p.omega_hang = g("omega_hang"); p.x_center = g("x_center"); p.center_tol = g("center_tol");
    p.engage_on_release = g("engage_on_release") != 0;
    return p;
}

}

// ---- golden: C++ must reproduce the Python reference (py/make_golden.py) ----

TEST(supervisor_golden_matches_python) {
    Csv pc = readCsv("tests/golden/supervisor_params.csv");
    Csv g = readCsv("tests/golden/supervisor_golden.csv");
    CHECK_EQ(pc.header.size(), 21u);   // case + 20 parameters: a new field must be added here too
    std::map<std::string, SupervisorParams> params;
    for (const auto& r : pc.rows) params[r[0]] = paramsFrom(pc, r);
    CHECK_EQ(params.size(), 9u);

    std::map<std::string, SupervisorState> state;
    std::map<int, int> eventsSeen;
    int mismatches = 0;
    for (const auto& r : g.rows) {
        auto b = [&](const char* n) { return r[static_cast<std::size_t>(g.col(n))] == "1"; };
        auto d = [&](const char* n) { return num(r[static_cast<std::size_t>(g.col(n))]); };
        SupervisorInputs in;
        in.theta = d("theta"); in.omega = d("omega"); in.x = d("x"); in.v = d("v");
        in.limit_pressed = b("limit_pressed"); in.axis_accelerating = b("axis_accelerating");
        in.overrun = b("overrun"); in.encoder_fault = b("encoder_fault"); in.arm = b("arm"); in.stop = b("stop");
        in.released = b("released");
        auto res = supervisorStep(params.at(r[0]), state[r[0]], in, d("dt"));
        state[r[0]] = res.state;
        const auto& o = res.out;
        if (static_cast<int>(o.mode) != static_cast<int>(d("mode")) || o.driver_enabled != b("driver") ||
            o.run_controller != b("run") || o.reset_controller != b("reset") || !close(o.a_brake_cmd, d("a_brake")) ||
            static_cast<int>(o.event) != static_cast<int>(d("event")))
            ++mismatches;
        ++eventsSeen[static_cast<int>(o.event)];
    }
    std::cout << "    golden: " << g.rows.size() << " supervisor samples, " << eventsSeen.size() - 1
              << " distinct events\n";
    CHECK_EQ(mismatches, 0);
    for (int e : {1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15}) CHECK(eventsSeen[e] > 0);   // all exercised
}

TEST(brake_distance_matches_python) {
    Csv c = readCsv("tests/golden/brake_distance.csv");
    CHECK_EQ(c.rows.size(), 5u);
    for (const auto& r : c.rows) CHECK(close(brakeDistance(num(r[0]), num(r[1]), num(r[2]), num(r[3])), num(r[4])));
}

TEST(units_match_python) {
    Csv c = readCsv("tests/golden/units.csv");
    CHECK_EQ(c.rows.size(), 4u);
    for (const auto& r : c.rows) {
        Calib cal;
        cal.theta_sign = std::atoi(r[1].c_str());
        cal.c_bottom = num(r[2]);
        cal.x_sign = std::atoi(r[4].c_str());
        CHECK(close(thetaFromCounts(std::atoll(r[0].c_str()), cal), num(r[7])));
        CHECK(close(xFromPos(std::atoll(r[3].c_str()), cal), num(r[8])));
        CHECK(close(vFromAxisSpeed(std::atoll(r[5].c_str()), cal), num(r[9])));
        CHECK_EQ(axisAccelArg(num(r[6]), cal), std::atoi(r[10].c_str()));
    }
}

// ---- properties ----

TEST(brake_distance_closed_form_matches_integration) {
    for (double v0 : {0.2, 0.8, 1.5, 3.0}) {
        for (double ab : {1.0, 3.0, 6.0}) {
            const double A = 4.5, V = 4.5;
            double v = v0, x = 0, h = 1e-6;
            while (v > 0) {                       // decel = min(a_brake, A(1 - v/V)), Euler in time
                const double a = std::min(ab, A * (1 - v / V));
                x += v * h;
                v -= a * h;
            }
            CHECK_NEAR(brakeDistance(v0, ab, A, V), x, 1e-4 * std::max(1.0, x));
        }
    }
    CHECK_EQ(brakeDistance(0.0, 3, 4.5, 4.5), 0.0);
    CHECK(std::isinf(brakeDistance(5.0, 3, 4.5, 4.5)));   // faster than the Axis cap allows to brake
}

TEST(units_wrap_and_conventions) {
    const double pi = 3.141592653589793;
    CHECK_NEAR(wrap(pi), pi, 1e-15);
    CHECK_NEAR(wrap(-pi), pi, 1e-15);
    CHECK_NEAR(wrap(3 * pi / 2), -pi / 2, 1e-15);
    Calib c;   // c_bottom = 0, theta_sign = +1
    // count 1199 covers [1199, 1200): centre 1199.5 -> half a count before upright
    CHECK_NEAR(thetaFromCounts(1199, c), -0.5 * 2 * pi / 2400, 1e-12);
    CHECK_NEAR(std::fabs(thetaFromCounts(0, c)), pi - 0.5 * 2 * pi / 2400, 1e-12);   // hanging
    // +x acceleration -> negative Axis argument (internal sign), x2 toggles
    CHECK_EQ(axisAccelArg(1.0, c), -26667);
    // Axis curSpeed -26667 toggles/s = +1 m/s
    CHECK_NEAR(vFromAxisSpeed(-26667, c), 1.0, 1e-4);
}

TEST(supervisor_engage_needs_consecutive_samples) {
    SupervisorParams p;
    SupervisorState s;
    SupervisorInputs hang;
    hang.theta = 3.13;
    hang.arm = true;
    auto r = supervisorStep(p, s, hang, 0.002);
    CHECK(r.out.mode == Mode::Armed);
    SupervisorInputs up;
    up.theta = 0.01;
    for (int k = 0; k < p.engage_samples - 1; ++k) r = supervisorStep(p, r.state, up, 0.002);
    CHECK(r.out.mode == Mode::Armed);
    SupervisorInputs fast = up;
    fast.omega = 1.0;                             // one fast sample resets the count
    r = supervisorStep(p, r.state, fast, 0.002);
    r = supervisorStep(p, r.state, up, 0.002);
    CHECK(r.out.mode == Mode::Armed);
    for (int k = 0; k < p.engage_samples; ++k) r = supervisorStep(p, r.state, up, 0.002);
    CHECK(r.out.mode == Mode::Engaged);
}

TEST(supervisor_brake_command_never_overshoots) {
    SupervisorParams p;
    SupervisorState s;
    s.mode = Mode::Stopping;
    SupervisorInputs in;
    in.v = 0.004 + p.v_stopped;      // small speed: command limited to v/dt
    auto r = supervisorStep(p, s, in, 0.002);
    CHECK_NEAR(r.out.a_brake_cmd, -in.v / 0.002 < -p.a_brake ? -p.a_brake : -in.v / 0.002, 1e-12);
    in.v = -0.5;
    r = supervisorStep(p, s, in, 0.002);
    CHECK_NEAR(r.out.a_brake_cmd, p.a_brake, 1e-12);
}

TEST(supervisor_names_cover_all_reasons) {
    for (int i = 0; i <= 15; ++i) CHECK(std::string(reasonName(static_cast<Reason>(i))) != "?");
    CHECK_EQ(std::string(modeName(Mode::Stopping)), std::string("STOPPING"));
}

TEST(supervisor_engage_on_release_needs_the_release) {
    SupervisorParams p;
    p.engage_on_release = true;
    SupervisorState st;
    SupervisorInputs in;
    in.theta = 3.141592653589793;                                 // hanging, still: arm
    in.arm = true;
    st = supervisorStep(p, st, in, 0.002).state;
    CHECK(st.mode == Mode::Armed);
    in.arm = false;
    in.theta = 0.02;                                              // near upright and still, but held
    for (int k = 0; k < 200; ++k) st = supervisorStep(p, st, in, 0.002).state;
    CHECK(st.mode == Mode::Armed);                                // the old rule would have engaged
    in.released = true;
    in.theta = 0.09;                                              // released, but beyond theta_engage
    st = supervisorStep(p, st, in, 0.002).state;
    CHECK(st.mode == Mode::Armed);
    in.theta = 0.03;
    const SupervisorResult r = supervisorStep(p, st, in, 0.002);
    CHECK(r.state.mode == Mode::Engaged);
    CHECK(r.out.reset_controller);
    CHECK(r.out.event == Reason::EngagedEvent);
}
