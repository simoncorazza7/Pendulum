#include "hw/pi_hw.h"

#include <cmath>
#include <stdexcept>

#include "rt/rt.h"

namespace hw {

PiHw::PiHw(const std::string& chipName, const PiPins& pins, bool allowRealChip, int axisMaxAccelToggles) {
    chip_ = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip_) throw std::runtime_error("cannot open " + chipName);
    label_ = gpiod_chip_label(chip_);
    if (label_ != "gpio-mockup-A" && !allowRealChip) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        throw std::runtime_error("refusing " + chipName + " ('" + label_ +
                                 "'): the real rig needs allow_real_chip=true");
    }
    // Bring-up mode: no homing, EN stays 1 (driver off) until enableDriver().
    axis_ = std::make_unique<Axis>(chip_, pins.step, pins.dir, pins.en, pins.limit_right, pins.limit_left,
                                   pins.ms1, pins.ms2, false);
    axis_->setRawPosition(true);                   // pendctl owns limits; pos only counts steps
    axis_->setPositionReference(0, 1 << 30);       // pos 0 = where the cart is at start
    axis_->setMaxAcceleration(axisMaxAccelToggles);
    reader_ = std::make_unique<EdgeReader>(chip_, pins.enc_a, pins.enc_b);
}

PiHw::~PiHw() {
    if (axis_) {
        axis_->halt();
        for (int i = 0; i < 500 && !axis_->isStopped(); ++i) rt::sleepUntilNs(rt::monoNs() + 1000000);
        axis_->disableDriver();
    }
    if (reader_) reader_->stop();
    axis_.reset();                                 // joins the step thread, releases its lines
    reader_.reset();
    if (chip_) gpiod_chip_close(chip_);
}

bool PiHw::start(const PiRt& rt, std::string* err) {
    if (rt.step_prio > 0 && !axis_->configureRealtime(rt.step_prio, rt.step_cpu)) {
        if (err) *err = "Axis step thread: SCHED_FIFO failed (run with sudo)";
        return false;
    }
    return reader_->start(rt.enc_prio, rt.enc_cpu, err);
}

Sample PiHw::read() {
    Sample s;
    s.t_ns = rt::monoNs();
    s.counts = reader_->counts();
    s.n_edges = reader_->drainEdges(s.edges.data(), Sample::kMaxEdges, &s.edges_overflow);
    s.pos_usteps = axis_->getPos();
    s.speed_toggles = axis_->getSpeed();
    s.limit_right = axis_->limitRightPressed();
    s.limit_left = axis_->limitLeftPressed();
    s.axis_accelerating = axis_->isAccelerating();
    s.encoder_fault = reader_->fault();
    s.driver_enabled = axis_->isDriverEnabled();
    return s;
}

void PiHw::commandAccel(int axisArg) {
    if (!halted_) axis_->setAcceleration(axisArg);
}

void PiHw::enableDriver(bool on) {
    if (on) axis_->enableDriver(); else axis_->disableDriver();
}

void PiHw::halt() {
    halted_ = true;
    axis_->halt();
}

bool PiHw::isStopped() { return axis_->isStopped(); }

}

namespace hw {

void PiHw::setCommandWatchdog(double timeoutS, int brakeToggles2) {
    axis_->setCommandWatchdog(static_cast<int>(std::lround(timeoutS * 1e6)), brakeToggles2);
}

}
