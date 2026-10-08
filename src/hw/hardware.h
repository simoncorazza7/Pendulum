#ifndef PENDEL_HW_HARDWARE_H
#define PENDEL_HW_HARDWARE_H

#include <array>
#include <cstdint>

// Thin hardware interface for the pendctl runtime (Gate A architecture). It sits exactly at the
// hardware boundary, so everything here is in raw units: encoder counts, edge timestamps, Axis
// microsteps, STEP toggles and the Axis's internal sign. The runtime converts with core::units.
// Implementations: SimHw (simulated plant, sim_hw.h) and, later, PiHw (Axis + encoder on the rig).
namespace hw {

struct Edge {
    std::int64_t ts_ns;          // edge timestamp, ns (same clock as Sample::t_ns)
    std::int64_t count_after;    // cumulative count after this edge
};

struct Sample {
    static constexpr int kMaxEdges = 64;   // per sample; 2 rev/s at 2 ms is ~10

    std::int64_t t_ns = 0;                 // sample time, ns
    std::int64_t counts = 0;               // cumulative encoder count (0 at program start)
    std::array<Edge, kMaxEdges> edges {};  // edges since the previous read(), oldest first
    int n_edges = 0;
    bool edges_overflow = false;           // more than kMaxEdges since the last read (some dropped)
    std::int64_t pos_usteps = 0;           // Axis pos (microsteps; 0 side = "right" switch)
    std::int64_t speed_toggles = 0;        // Axis curSpeed (STEP toggles/s, internal sign)
    bool limit_right = false;              // pressed (switch at the low-pos end, GPIO2 on the rig)
    bool limit_left = false;               // pressed (switch at the high-pos end, GPIO9)
    bool axis_accelerating = false;        // Axis is following ACCELERATE commands
    bool encoder_fault = false;            // decoder saw an impossible transition
    bool driver_enabled = false;
};

class Hardware {
public:
    virtual ~Hardware() = default;
    virtual Sample read() = 0;                   // once per control period
    virtual void commandAccel(int axisArg) = 0;  // Axis::setAcceleration units (toggles/s^2, internal sign)
    virtual void enableDriver(bool on) = 0;
    virtual void halt() = 0;                     // brake to standstill, ignore further commands
    virtual bool isStopped() = 0;
};

}

#endif
