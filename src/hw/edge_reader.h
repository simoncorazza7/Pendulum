#ifndef PENDEL_HW_EDGE_READER_H
#define PENDEL_HW_EDGE_READER_H

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <gpiod.h>

#include "hw/hardware.h"

// Encoder reader for the pendctl runtime (replaces the old Encoder class there; main keeps it).
// A thread waits for edge events on both channels, reads everything pending, merges the two
// channels in timestamp order and decodes each edge immediately from its kernel edge type.
// Evidence (docs/stage0_6_7_10_results.md): the edge type was never wrong in ~0.7 M events, and
// the type decoder recovers from a lost edge; the old decoder's "wait for both queues" holds back
// the newest edge.
//   counts()      cumulative count, same sign convention as the old decoder (A leading = negative)
//   drainEdges()  edges since the last call (single consumer), with kernel timestamps
//   fault()       sticky: an edge that changed nothing (same type twice = lost edge) or a read that
//                 returned a full 16-event kernel queue (edges may have been dropped)
namespace hw {

class EdgeReader {
public:
    EdgeReader(gpiod_chip* chip, unsigned lineA, unsigned lineB);
    ~EdgeReader();

    // Claims the lines (call after memory/RT setup of the process), reads the start levels and
    // starts the thread; prio <= 0: normal scheduling. Returns false with *err on failure.
    bool start(int prio, int cpu, std::string* err);
    void stop();

    std::int64_t counts() const { return count_.load(std::memory_order_acquire); }   // 32-bit inside: lock-free on armhf
    int drainEdges(Edge* out, int maxEdges, bool* overflow);
    bool fault() const { return fault_.load(); }
    std::uint32_t events() const { return events_.load(); }
    std::uint32_t sameType() const { return sameType_.load(); }
    std::uint32_t fullBatches() const { return fullBatches_.load(); }

private:
    void loop();

    gpiod_chip* chip_;
    unsigned lineA_, lineB_;
    gpiod_line* a_ = nullptr;
    gpiod_line* b_ = nullptr;
    std::thread thread_;
    std::atomic<bool> running_{false};
    int state_[2] = {0, 0};                            // current levels of A and B (decoder state)
    std::int64_t initNs_ = 0;                          // CLOCK_MONOTONIC when the start levels were read
    std::atomic<std::int32_t> count_{0};               // +-2.1e9 counts; 64-bit atomics are not lock-free here
    std::atomic<bool> fault_{false};
    std::atomic<std::uint32_t> events_{0}, sameType_{0}, fullBatches_{0};

    static constexpr std::size_t kRing = 1024;         // single producer (thread), single consumer
    std::array<Edge, kRing> ring_ {};
    std::atomic<std::size_t> head_{0}, tail_{0};
    std::atomic<bool> ringOverflow_{false};
};

}

#endif
