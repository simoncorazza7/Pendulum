#ifndef PENDEL_CORE_SUPERVISOR_H
#define PENDEL_CORE_SUPERVISOR_H

// Supervisor state machine (brief §6), mirror of py/pendel/supervisor.py.
// IDLE -> ARMED -> ENGAGED -> STOPPING -> IDLE. Pure: params + state + inputs + dt -> output + state.
// The runtime enables the driver while output.driver_enabled, runs the controller while
// run_controller, resets it on reset_controller, and commands a_brake_cmd in STOPPING.
namespace core {

enum class Mode { Idle = 0, Armed = 1, Engaged = 2, Stopping = 3 };

enum class Reason {
    None = 0,
    ManualStop = 1,
    LimitSwitch = 2,
    EncoderFault = 3,
    AxisNotAccelerating = 4,
    Overrun = 5,
    ThetaAbort = 6,
    SoftLimit = 7,
    MaxRunTime = 8,
    Stopped = 9,          // STOPPING finished normally
    StopTimeout = 10,     // STOPPING gave up waiting for standstill
    ArmNotHanging = 11,
    ArmNotCentred = 12,
    ArmMoving = 13,
    EngagedEvent = 14,    // informational: ARMED -> ENGAGED
    ArmedEvent = 15,      // informational: IDLE -> ARMED
};

const char* modeName(Mode m);
const char* reasonName(Reason r);

struct SupervisorParams {
    double theta_engage = 0.06981317007977318;  // rad (4 deg)
    double omega_engage = 0.5;                  // rad/s
    int engage_samples = 25;                    // consecutive samples (50 ms at 2 ms)
    double theta_abort = 0.3839724354387525;    // rad (22 deg)
    double x_min = -0.20;                       // m, soft limit (PLACEHOLDER until 0.8)
    double x_max = 0.20;                        // m
    double a_brake = 3.0;                       // m/s^2, controlled-stop deceleration
    double t_react = 0.006;                     // s, reaction time added to the braking distance
    double brake_margin = 0.01;                 // m
    double axis_a = 4.5;                        // m/s^2, Axis maxAcceleration in SI (cap A(1-|v|/V))
    double axis_v = 4.5;                        // m/s, Axis maxSpeed in SI
    int overrun_streak = 3;                     // consecutive loop overruns -> abort
    double max_run_time = 60.0;                 // s engaged
    double v_stopped = 0.005;                   // m/s, standstill threshold
    double stop_timeout = 2.0;                  // s in STOPPING before giving up
    double hang_tol = 0.17453292519943295;      // rad from hanging, to arm (10 deg)
    double omega_hang = 0.3;                    // rad/s, to arm
    double x_center = 0.0;                      // m
    double center_tol = 0.02;                   // m, to arm
    bool engage_on_release = false;             // engage on the runtime's release detection (free motion
                                                // away from upright) instead of engage_samples still samples
};

struct SupervisorInputs {
    double theta = 0;               // rad from upright, wrapped
    double omega = 0;               // rad/s
    double x = 0;                   // m
    double v = 0;                   // m/s
    bool limit_pressed = false;
    bool axis_accelerating = true;  // Axis is executing ACCELERATE (not its own brake/stop)
    bool overrun = false;           // this loop iteration overran
    bool encoder_fault = false;
    bool arm = false;               // operator request (one sample)
    bool stop = false;              // operator request (one sample)
    bool released = false;          // runtime: the pendulum moves freely away from upright after a still hold
};

struct SupervisorState {
    Mode mode = Mode::Idle;
    int engage_count = 0;
    int overrun_count = 0;
    double t_mode = 0;              // s in the current mode
};

struct SupervisorOutput {
    Mode mode;
    bool driver_enabled;
    bool run_controller;
    bool reset_controller;
    double a_brake_cmd;             // m/s^2, valid in STOPPING (else 0)
    Reason event;                   // reason of a transition or refusal this sample
};

struct SupervisorResult {
    SupervisorOutput out;
    SupervisorState state;
};

SupervisorResult supervisorStep(const SupervisorParams& p, const SupervisorState& s,
                                const SupervisorInputs& in, double dt);

// Distance (m) to stop from |v| decelerating at a_brake, limited by the Axis cap A(1 - |v|/V).
double brakeDistance(double v, double a_brake, double axis_a, double axis_v);
// brakeDistance + reaction distance + margin, as used for the soft-limit check.
double stoppingDistance(const SupervisorParams& p, double v);

}

#endif
