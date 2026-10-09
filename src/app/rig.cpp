#include "app/rig.h"

#include <cmath>
#include <cstdio>
#include <iostream>

#include "app/modes.h"
#include "rt/rt.h"

namespace app {

hw::PiPins pinsFrom(const Config& c) {
    hw::PiPins p;
    p.step = static_cast<unsigned>(c.getInt("pins.step", p.step));
    p.dir = static_cast<unsigned>(c.getInt("pins.dir", p.dir));
    p.en = static_cast<unsigned>(c.getInt("pins.en", p.en));
    p.limit_right = static_cast<unsigned>(c.getInt("pins.limit_right", p.limit_right));
    p.limit_left = static_cast<unsigned>(c.getInt("pins.limit_left", p.limit_left));
    p.ms1 = static_cast<unsigned>(c.getInt("pins.ms1", p.ms1));
    p.ms2 = static_cast<unsigned>(c.getInt("pins.ms2", p.ms2));
    p.enc_a = static_cast<unsigned>(c.getInt("pins.enc_a", p.enc_a));
    p.enc_b = static_cast<unsigned>(c.getInt("pins.enc_b", p.enc_b));
    return p;
}

hw::PiRt rtFrom(const Config& c) {
    const bool on = c.getBool("rt.enable");
    hw::PiRt r;
    r.step_prio = on ? static_cast<int>(c.getInt("rt.step_prio")) : 0;
    r.step_cpu = static_cast<int>(c.getInt("rt.step_cpu"));
    r.enc_prio = on ? static_cast<int>(c.getInt("rt.enc_prio")) : 0;
    r.enc_cpu = static_cast<int>(c.getInt("rt.enc_cpu"));
    return r;
}

TickClock::TickClock(double ts) : period_(std::llround(ts * 1e9)) {}

std::int64_t TickClock::wait() {
    if (k_ == 0) start_ = rt::monoNs() + 10 * period_;
    const std::int64_t deadline = start_ + std::int64_t(k_) * period_;
    rt::sleepUntilNs(deadline);
    late_ = rt::monoNs() - deadline;
    ++k_;
    return late_;
}

bool calibrateAtStart(hw::Hardware& hw, rt::SwingCalibrator& cal, TickClock& clk, double timeout,
                      const std::function<void(const hw::Sample&, std::int64_t)>& onTick) {
    const double degPerCount = 360.0 / 2400.0;
    std::cout << "calib: give the HANGING pendulum a small swing (3-8 deg) and let go. Hands off until "
                 "'calib: accepted' (~5 s).\n"
              << std::flush;
    const double t0 = clk.seconds();
    double nextStatus = t0 + 1.0;
    while (!flags.stop) {
        const std::int64_t late = clk.wait();
        const hw::Sample s = hw.read();
        onTick(s, late);
        const bool done = cal.update(s);
        const rt::SwingCalibStatus& st = cal.status();
        if (done) {
            char b[200];
            std::snprintf(b, sizeof b, "calib: accepted c_bottom = %.3f counts (se %.3f, rms %.2f, amplitude %.1f deg)",
                          st.c_bottom, st.fit.se, st.fit.rms, st.amplitude * degPerCount);
            std::cout << b << "\n" << std::flush;
            return true;
        }
        if (clk.seconds() >= nextStatus) {
            nextStatus += 1.0;
            char b[240];
            std::snprintf(b, sizeof b, "calib: %4.0f s  amplitude %4.1f deg, %4d edges, rms %5.2f, se %5.3f: %s",
                          clk.seconds() - t0, st.amplitude * degPerCount, st.n_edges, st.fit.ok ? st.fit.rms : 0.0,
                          st.fit.ok ? st.fit.se : 0.0, st.why);
            std::cout << b << "\n" << std::flush;
        }
        if (clk.seconds() - t0 > timeout) {
            std::cout << "calib: no accepted fit within " << timeout << " s; giving up (nothing was moved)\n";
            return false;
        }
    }
    return false;
}

}
