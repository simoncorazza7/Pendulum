#ifndef PENDEL_RUNTIME_PARAMS_H
#define PENDEL_RUNTIME_PARAMS_H

#include "common/config.h"
#include "core/units.h"
#include "runtime/actuator.h"
#include "runtime/balance.h"
#include "runtime/damp.h"
#include "runtime/session.h"
#include "runtime/swingup.h"
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
// Balance session (pendctl --mode balance): balanceFrom with the reduced balance.a_max / v_max, the
// mover from balance.* and move.* (tracking), the homing checks. Throws on unusable values, and if the
// soft limits plus braking distance do not fit inside the switches.
SessionConfig sessionFrom(const Config& cfg);
// Swing-up runtime from swingup.* plus the session's balance (gains, reduced a_max / v_max, estimator, limits).
// calib.c_bottom and calib.pos_center are set by the caller (the session).
SwingUpConfig swingUpFrom(const Config& cfg);
// Stage 0.9 actuator test (pendctl --mode actuator): actuator.* plus homing, travel and touches from balance.* and
// move.* (as the balance session). The table of profile `table` is loaded by the caller (loadActTable).
// Throws on unusable values, including a ladder in which the Axis cap would bite.
ActuatorConfig actuatorFrom(const Config& cfg);

}

#endif
