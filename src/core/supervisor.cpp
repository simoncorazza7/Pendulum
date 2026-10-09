#include "core/supervisor.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/units.h"

namespace core {
namespace {

constexpr double kPi = 3.141592653589793;

Reason commonAbort(const SupervisorParams& p, const SupervisorInputs& in, int overruns) {
    if (in.stop) return Reason::ManualStop;
    if (in.limit_pressed) return Reason::LimitSwitch;
    if (in.encoder_fault) return Reason::EncoderFault;
    if (!in.axis_accelerating) return Reason::AxisNotAccelerating;
    if (overruns >= p.overrun_streak) return Reason::Overrun;
    return Reason::None;
}

bool softLimit(const SupervisorParams& p, const SupervisorInputs& in) {
    if (in.x >= p.x_max || in.x <= p.x_min) return true;
    const double d = stoppingDistance(p, in.v);
    return (in.v > 0 && in.x + d >= p.x_max) || (in.v < 0 && in.x - d <= p.x_min);
}

Reason armRefusal(const SupervisorParams& p, const SupervisorInputs& in) {
    if (in.limit_pressed) return Reason::LimitSwitch;
    if (in.encoder_fault) return Reason::EncoderFault;
    if (std::fabs(wrap(in.theta)) < kPi - p.hang_tol) return Reason::ArmNotHanging;
    if (std::fabs(in.omega) > p.omega_hang || std::fabs(in.v) > p.v_stopped) return Reason::ArmMoving;
    if (std::fabs(in.x - p.x_center) > p.center_tol) return Reason::ArmNotCentred;
    return Reason::None;
}

}

double brakeDistance(double v, double a_brake, double axis_a, double axis_v) {
    v = std::fabs(v);
    if (v == 0.0) return 0.0;
    if (v >= axis_v) return std::numeric_limits<double>::infinity();
    const double v_star = a_brake < axis_a ? axis_v * (1 - a_brake / axis_a) : 0.0;   // cap binds above
    if (v <= v_star) return v * v / (2 * a_brake);
    auto F = [&](double u) { return (axis_v / axis_a) * (-u - axis_v * std::log(1 - u / axis_v)); };
    return v_star * v_star / (2 * a_brake) + F(v) - F(v_star);
}

double stoppingDistance(const SupervisorParams& p, double v) {
    return brakeDistance(v, p.a_brake, p.axis_a, p.axis_v) + std::fabs(v) * p.t_react + p.brake_margin;
}

SupervisorResult supervisorStep(const SupervisorParams& p, const SupervisorState& s,
                                const SupervisorInputs& in, double dt) {
    Mode mode = s.mode;
    int ec = s.engage_count;
    double t_mode = s.t_mode + dt;
    const int oc = in.overrun ? s.overrun_count + 1 : 0;
    Reason event = Reason::None;
    bool reset = false;

    switch (mode) {
        case Mode::Idle:
            if (in.arm) {
                event = armRefusal(p, in);
                if (event == Reason::None) {
                    mode = Mode::Armed; t_mode = 0; ec = 0; event = Reason::ArmedEvent;
                }
            }
            break;
        case Mode::Armed:
            event = commonAbort(p, in, oc);
            if (event == Reason::None && softLimit(p, in)) event = Reason::SoftLimit;
            if (event != Reason::None) {
                mode = Mode::Stopping; t_mode = 0;
            } else if (p.engage_on_release) {
                // Engage only once the pendulum moves freely (docs/stage1_drift.md): the cart never
                // accelerates while a hand still holds the rod, which would stay as cart momentum.
                if (in.released && std::fabs(in.theta) < p.theta_engage) {
                    mode = Mode::Engaged; t_mode = 0; reset = true; event = Reason::EngagedEvent;
                }
            } else {
                const bool near = std::fabs(in.theta) < p.theta_engage && std::fabs(in.omega) < p.omega_engage;
                ec = near ? ec + 1 : 0;
                if (ec >= p.engage_samples) {
                    mode = Mode::Engaged; t_mode = 0; reset = true; event = Reason::EngagedEvent;
                }
            }
            break;
        case Mode::Engaged:
            event = commonAbort(p, in, oc);
            if (event == Reason::None && std::fabs(in.theta) > p.theta_abort) event = Reason::ThetaAbort;
            if (event == Reason::None && softLimit(p, in)) event = Reason::SoftLimit;
            if (event == Reason::None && s.t_mode + dt > p.max_run_time) event = Reason::MaxRunTime;
            if (event != Reason::None) { mode = Mode::Stopping; t_mode = 0; }
            break;
        case Mode::Stopping:
            if (std::fabs(in.v) < p.v_stopped) {
                mode = Mode::Idle; t_mode = 0; event = Reason::Stopped;
            } else if (t_mode > p.stop_timeout) {
                mode = Mode::Idle; t_mode = 0; event = Reason::StopTimeout;
            }
            break;
    }

    double a_brake = 0.0;
    if (mode == Mode::Stopping && in.v != 0.0)
        a_brake = -std::copysign(std::min(p.a_brake, std::fabs(in.v) / dt), in.v);
    if (mode != Mode::Armed) ec = 0;

    SupervisorResult r;
    r.out = SupervisorOutput{mode, mode != Mode::Idle, mode == Mode::Engaged, reset, a_brake, event};
    r.state = SupervisorState{mode, ec, oc, t_mode};
    return r;
}

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Idle: return "IDLE";
        case Mode::Armed: return "ARMED";
        case Mode::Engaged: return "ENGAGED";
        case Mode::Stopping: return "STOPPING";
    }
    return "?";
}

const char* reasonName(Reason r) {
    switch (r) {
        case Reason::None: return "none";
        case Reason::ManualStop: return "manual stop";
        case Reason::LimitSwitch: return "limit switch";
        case Reason::EncoderFault: return "encoder fault";
        case Reason::AxisNotAccelerating: return "axis left ACCELERATE";
        case Reason::Overrun: return "loop overrun streak";
        case Reason::ThetaAbort: return "angle beyond theta_abort";
        case Reason::SoftLimit: return "soft limit within braking distance";
        case Reason::MaxRunTime: return "max run time";
        case Reason::Stopped: return "stopped";
        case Reason::StopTimeout: return "stop timeout";
        case Reason::ArmNotHanging: return "arm refused: pendulum not hanging";
        case Reason::ArmNotCentred: return "arm refused: cart not centred";
        case Reason::ArmMoving: return "arm refused: pendulum or cart moving";
        case Reason::EngagedEvent: return "engaged";
        case Reason::ArmedEvent: return "armed";
    }
    return "?";
}

}
