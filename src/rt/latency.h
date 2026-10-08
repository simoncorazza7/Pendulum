#ifndef PENDEL_LATENCY_H
#define PENDEL_LATENCY_H

#include <array>
#include <cstdint>
#include <fstream>
#include <string>

// Fixed-bin latency histogram (Stage 0.5): 1 us bins from 0 to kBins-1 us, plus overflow.
// add() is real-time safe. One thread adds; read after it has stopped.
class LatencyHist {
public:
    static constexpr int kBins = 5000;   // 0 .. 4999 us

    void add(std::int64_t ns) noexcept {
        if (ns < 0) { ++negative_; ns = 0; }
        std::int64_t us = ns / 1000;
        if (us < kBins) ++bins_[static_cast<std::size_t>(us)]; else ++overflow_;
        ++count_;
        if (ns > maxNs_) maxNs_ = ns;
    }

    std::uint64_t count() const { return count_; }
    std::uint64_t overflow() const { return overflow_; }   // samples >= kBins us
    std::uint64_t negative() const { return negative_; }   // early samples, counted as 0
    std::int64_t maxNs() const { return maxNs_; }

    // Upper edge (us) of the bin holding quantile q (0..1); kBins if it lies in the overflow.
    int quantileUs(double q) const {
        if (count_ == 0) return 0;
        std::uint64_t target = static_cast<std::uint64_t>(q * double(count_ - 1)) + 1, acc = 0;
        for (int i = 0; i < kBins; ++i) {
            acc += bins_[static_cast<std::size_t>(i)];
            if (acc >= target) return i + 1;
        }
        return kBins;
    }

    // "bin_us,count" for non-empty bins; the overflow bin is written as bin_us = kBins.
    void writeCsv(const std::string& path) const {
        std::ofstream out(path);
        out << "bin_us,count\n";
        for (int i = 0; i < kBins; ++i)
            if (bins_[static_cast<std::size_t>(i)]) out << i << ',' << bins_[static_cast<std::size_t>(i)] << '\n';
        if (overflow_) out << kBins << ',' << overflow_ << '\n';
    }

private:
    std::array<std::uint64_t, kBins> bins_ {};
    std::uint64_t count_ = 0, overflow_ = 0, negative_ = 0;
    std::int64_t maxNs_ = 0;
};

#endif
