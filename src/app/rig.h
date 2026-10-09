#ifndef PENDEL_APP_RIG_H
#define PENDEL_APP_RIG_H

#include <cstdint>
#include <functional>

#include "common/config.h"
#include "hw/hardware.h"
#include "hw/pi_hw.h"
#include "runtime/swing_calib.h"

// Pi-side helpers shared by the pendctl rig modes (move, shadow, damp).
namespace app {

hw::PiPins pinsFrom(const Config& c);        // pins.* (defaults: the old hand wiring)
hw::PiRt rtFrom(const Config& c);            // rt.*; priorities 0 if rt.enable = false

// Fixed-rate loop on absolute CLOCK_MONOTONIC deadlines.
class TickClock {
public:
    explicit TickClock(double ts);
    std::int64_t wait();                      // sleeps until the next deadline; returns the lateness, ns
    std::uint32_t tick() const { return k_ - 1; }   // index of the tick that wait() just started
    bool overrun() const { return late_ > period_; }
    double seconds() const { return (k_ - 1) * period_ * 1e-9; }   // since the first tick

private:
    std::int64_t period_, start_ = 0, late_ = 0;
    std::uint32_t k_ = 0;
};

// Start-up c_bottom calibration (rt::SwingCalibrator) inside the loop: prints the instruction and
// a status line every second, calls onTick(sample, lateNs) every tick (for logging). Returns true
// once a fit is accepted; false on timeout or a stop signal. Never commands the hardware.
bool calibrateAtStart(hw::Hardware& hw, rt::SwingCalibrator& cal, TickClock& clk, double timeout,
                      const std::function<void(const hw::Sample&, std::int64_t)>& onTick);

}

#endif
