#ifndef PENDEL_RUNTIME_PARAMS_H
#define PENDEL_RUNTIME_PARAMS_H

#include "common/config.h"
#include "core/units.h"
#include "runtime/balance.h"
#include "runtime/damp.h"
#include "runtime/swing_calib.h"

// Config -> runtime parameters (portable, unit-tested against config/pendctl.conf). Angles that a
// person edits are in degrees in the config (keys *_deg) and converted here; everything else is SI.
namespace rt {

// Signs, steps/m, toggles/step and cpr; c_bottom = 0 (set from the start-up fit), pos_center = 0
// (the app sets it to the Axis position at start).
core::Calib calibFrom(const Config& cfg);
SwingCalibParams swingCalibFrom(const Config& cfg);
// Gains by pole placement from plant.* and ctrl.*; throws std::runtime_error if core::validate fails.
BalanceConfig balanceFrom(const Config& cfg);
DampConfig dampFrom(const Config& cfg);

}

#endif
