#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

#include "check.h"
#include "rt/ringlog.h"

namespace {
std::vector<int> contents(const RingLog<int>& log) {
    std::vector<int> v;
    log.forEach([&](int x) { v.push_back(x); });
    return v;
}
}

TEST(ringlog_keeps_oldest_and_counts_drops) {
    RingLog<int> log(3, false);
    for (int i = 1; i <= 5; ++i) log.push(i);
    CHECK_EQ(log.size(), 3u);
    CHECK_EQ(log.dropped(), 2u);
    CHECK(contents(log) == (std::vector<int>{1, 2, 3}));
}

TEST(ringlog_overwrite_keeps_newest_in_order) {
    RingLog<int> log(3, true);
    for (int i = 1; i <= 5; ++i) log.push(i);
    CHECK_EQ(log.size(), 3u);
    CHECK_EQ(log.overwritten(), 2u);
    CHECK_EQ(log.dropped(), 0u);
    CHECK(contents(log) == (std::vector<int>{3, 4, 5}));
}

TEST(ringlog_partial_fill) {
    RingLog<int> log(4);
    log.push(7);
    log.push(8);
    CHECK(contents(log) == (std::vector<int>{7, 8}));
}

TEST(ringlog_zero_capacity_throws) {
    CHECK_THROWS(RingLog<int>(0));
}

TEST(ringlog_csv_roundtrip) {
    RingLog<int> log(10);
    for (int i = 0; i < 4; ++i) log.push(i * i);
    const char* path = "build/test_ringlog.csv";
    writeCsv(log, path, "sq", [](std::ostream& out, int x) { out << x; });
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    CHECK_EQ(ss.str(), std::string("sq\n0\n1\n4\n9\n"));
    std::remove(path);
}
