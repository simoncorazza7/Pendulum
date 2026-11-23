#ifndef PENDEL_HW_PI_HW_H
#define PENDEL_HW_PI_HW_H

#include <memory>
#include <string>

#include <gpiod.h>

#include "axis.h"
#include "hw/edge_reader.h"
#include "hw/hardware.h"

// The real rig behind hw::Hardware (Pi only). Wraps the existing Axis (bring-up mode: no homing,
// EN high until enableDriver(); raw position: pendctl owns all limits) and an EdgeReader for the
// pendulum encoder. Pins come from the config (the new PCB may change them).
namespace hw {

struct PiPins {
    unsigned step = 27, dir = 22, en = 24, limit_right = 2, limit_left = 9, ms1 = 20, ms2 = 21;
    unsigned enc_a = 10, enc_b = 19;
};

struct PiRt {
    int step_prio = 90, step_cpu = 3;     // Axis step thread
    int enc_prio = 85, enc_cpu = 2;       // encoder reader
};

class PiHw : public Hardware {
public:
    // Opens the chip. Refuses (throws) a chip that is not gpio-mockup unless allowRealChip.
    PiHw(const std::string& chipName, const PiPins& pins, bool allowRealChip, int axisMaxAccelToggles);
    ~PiHw() override;

    bool start(const PiRt& rt, std::string* err);   // Axis thread already runs; starts the reader, sets RT

    Sample read() override;
    void commandAccel(int axisArg) override;
    void enableDriver(bool on) override;
    void halt() override;
    bool isStopped() override;

    Axis* axis() { return axis_.get(); }
    EdgeReader& encoder() { return *reader_; }
    // Stall guard in the Axis step thread (Axis::setCommandWatchdog): without a new command for timeoutS, brake
    // to standstill at brakeToggles2 (STEP toggles/s^2). Off unless a mode sets it (pendctl --mode actuator).
    void setCommandWatchdog(double timeoutS, int brakeToggles2);
    std::uint32_t watchdogTrips() { return axis_->getWatchdogTrips(); }
    std::string chipLabel() const { return label_; }

private:
    gpiod_chip* chip_ = nullptr;
    std::string label_;
    std::unique_ptr<Axis> axis_;
    std::unique_ptr<EdgeReader> reader_;
    bool halted_ = false;
};

}

#endif
