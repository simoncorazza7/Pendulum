#ifndef PENDEL_HW_SENSOR_HW_H
#define PENDEL_HW_SENSOR_HW_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <gpiod.h>

#include "hw/edge_reader.h"
#include "hw/hardware.h"
#include "hw/pi_hw.h"

// Sensors only (shadow mode, Pi only): the pendulum encoder (EdgeReader) and the two limit switches
// as plain inputs. It never requests STEP, DIR, EN or the microstep lines, so a process using it
// cannot energise the motor or move the cart: EN stays as the boot config / last owner left it
// (high = driver off). Motor commands are counted and ignored. pos and speed read as 0.
namespace hw {

class SensorHw : public Hardware {
public:
    // Opens the chip. Refuses (throws) a chip that is not gpio-mockup unless allowRealChip.
    SensorHw(const std::string& chipName, const PiPins& pins, bool allowRealChip);
    ~SensorHw() override;

    bool start(int encPrio, int encCpu, std::string* err);   // claims the switch lines, starts the reader

    Sample read() override;
    void commandAccel(int) override { ignored_++; }
    void enableDriver(bool on) override { if (on) ignored_++; }
    void halt() override {}
    bool isStopped() override { return true; }

    std::uint32_t ignoredCommands() const { return ignored_.load(); }
    EdgeReader& encoder() { return *reader_; }
    std::string chipLabel() const { return label_; }

private:
    gpiod_chip* chip_ = nullptr;
    std::string label_;
    PiPins pins_;
    gpiod_line* limR_ = nullptr;
    gpiod_line* limL_ = nullptr;
    std::unique_ptr<EdgeReader> reader_;
    std::atomic<std::uint32_t> ignored_{0};
};

}

#endif
