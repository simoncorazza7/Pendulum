#ifndef PENDEL_RINGLOG_H
#define PENDEL_RINGLOG_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

// Fixed-capacity record log for real-time loops (Stage 0.4).
//
// All memory is allocated (and touched) in the constructor, so push() never allocates, locks or
// does I/O. One thread pushes; read the log only after that thread has stopped.
//   overwrite = false: when full, new records are dropped (keeps the start of a run)
//   overwrite = true:  when full, the oldest record is overwritten (keeps the end of a run)
template <typename T>
class RingLog {
public:
    explicit RingLog(std::size_t capacity, bool overwrite = false)
        : buf_(capacity), overwrite_(overwrite) {
        if (capacity == 0) throw std::invalid_argument("RingLog capacity must be > 0");
    }

    bool push(const T& rec) noexcept {
        if (count_ == buf_.size()) {
            if (!overwrite_) { ++dropped_; return false; }
            buf_[head_] = rec;
            head_ = (head_ + 1) % buf_.size();
            ++overwritten_;
            return true;
        }
        buf_[(head_ + count_) % buf_.size()] = rec;
        ++count_;
        return true;
    }

    std::size_t size() const { return count_; }
    std::size_t capacity() const { return buf_.size(); }
    std::uint64_t dropped() const { return dropped_; }        // records lost because the log was full
    std::uint64_t overwritten() const { return overwritten_; }

    // Visits the stored records from oldest to newest.
    template <typename F>
    void forEach(F f) const {
        for (std::size_t i = 0; i < count_; ++i) f(buf_[(head_ + i) % buf_.size()]);
    }

private:
    std::vector<T> buf_;
    std::size_t head_ = 0;   // index of the oldest record
    std::size_t count_ = 0;
    std::uint64_t dropped_ = 0;
    std::uint64_t overwritten_ = 0;
    bool overwrite_;
};

// Writes header + one line per record. writeRow(out, rec) writes the comma-separated fields
// without the trailing newline.
template <typename T, typename RowWriter>
void writeCsv(const RingLog<T>& log, const std::string& path, const std::string& header,
              RowWriter writeRow) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open " + path);
    out << header << '\n';
    log.forEach([&](const T& rec) {
        writeRow(out, rec);
        out << '\n';
    });
    if (!out) throw std::runtime_error("write failed: " + path);
}

#endif
