#include "hw/sensor_hw.h"

#include <stdexcept>

#include "rt/rt.h"

namespace hw {

SensorHw::SensorHw(const std::string& chipName, const PiPins& pins, bool allowRealChip) : pins_(pins) {
    chip_ = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip_) throw std::runtime_error("cannot open " + chipName);
    label_ = gpiod_chip_label(chip_);
    if (label_ != "gpio-mockup-A" && !allowRealChip) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        throw std::runtime_error("refusing " + chipName + " ('" + label_ +
                                 "'): the real rig needs allow_real_chip=true (Simon's go)");
    }
    reader_ = std::make_unique<EdgeReader>(chip_, pins.enc_a, pins.enc_b);
}

SensorHw::~SensorHw() {
    if (reader_) reader_->stop();
    reader_.reset();
    if (limR_) gpiod_line_release(limR_);
    if (limL_) gpiod_line_release(limL_);
    if (chip_) gpiod_chip_close(chip_);
}

bool SensorHw::start(int encPrio, int encCpu, std::string* err) {
    limR_ = gpiod_chip_get_line(chip_, pins_.limit_right);
    limL_ = gpiod_chip_get_line(chip_, pins_.limit_left);
    if (!limR_ || !limL_ || gpiod_line_request_input(limR_, "pendctl-limit") < 0 ||
        gpiod_line_request_input(limL_, "pendctl-limit") < 0) {
        if (err) *err = "cannot request the limit switch lines as inputs";
        return false;
    }
    return reader_->start(encPrio, encCpu, err);
}

Sample SensorHw::read() {
    Sample s;
    s.t_ns = rt::monoNs();
    s.counts = reader_->counts();
    s.n_edges = reader_->drainEdges(s.edges.data(), Sample::kMaxEdges, &s.edges_overflow);
    s.pos_usteps = 0;
    s.speed_toggles = 0;
    s.limit_right = gpiod_line_get_value(limR_) == 0;   // NO switches with pull-ups: 0 = pressed
    s.limit_left = gpiod_line_get_value(limL_) == 0;
    s.axis_accelerating = false;
    s.encoder_fault = reader_->fault();
    s.driver_enabled = false;
    last_ = s;
    return s;
}

}
