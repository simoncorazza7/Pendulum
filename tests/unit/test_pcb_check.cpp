// PCB bring-up judgements (src/core/pcb_check.*): the board table, pull and bridge verdicts, encoder turn analysis,
// step modes, supply flags.

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "check.h"
#include "core/pcb_check.h"

using core::Level;
using core::LineLoad;

namespace {

bool has(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }

// A steady turn: n single steps of `dir`, the four quarters of each cycle taking q[0..3] ms (by lower count mod 4).
std::vector<core::EdgeSample> turn(std::int64_t count0, int n, int dir, const double q[4]) {
    std::vector<core::EdgeSample> e;
    std::int64_t c = count0, t = 1000000;
    for (int i = 0; i < n; ++i) {
        const std::int64_t next = c + dir;
        const std::int64_t lower = std::min(c, next);
        t += static_cast<std::int64_t>(q[((lower % 4) + 4) % 4] * 1e6);
        c = next;
        e.push_back({t, c});
    }
    return e;
}

}

TEST(pcb_board_table_matches_the_header) {
    const auto& lines = core::boardLines();
    CHECK_EQ(lines.size(), static_cast<std::size_t>(28));
    std::set<int> bcm, header;
    for (const auto& l : lines) {
        bcm.insert(l.bcm);
        header.insert(l.header);
        CHECK_EQ(core::headerPinName(l.header), "GPIO" + std::to_string(l.bcm));
    }
    CHECK_EQ(bcm.size(), static_cast<std::size_t>(28));
    CHECK_EQ(header.size(), static_cast<std::size_t>(28));
    // the pins the rig uses (config pins.*): STEP 27, DIR 22, EN 24, MS1 20, MS2 21, switches 2 / 9, encoder 10 / 19
    CHECK(core::boardLine(27)->output && core::boardLine(22)->output && core::boardLine(24)->output);
    CHECK(core::boardLine(20)->load == LineLoad::WeakPullDown && core::boardLine(21)->load == LineLoad::PullDown50k);
    for (int b : {2, 3, 9, 10, 19}) CHECK(core::boardLine(b)->load == LineLoad::PullUp);
    CHECK(core::boardLine(4)->load == LineLoad::None);
    CHECK_EQ(core::headerPinName(1), "3V3");
    CHECK_EQ(core::headerPinName(4), "5V");
    CHECK_EQ(core::headerPinName(39), "GND");
    // GPIO2 (pin 3) sits between 3V3 (1), 5V (4) and GPIO3 (5)
    const std::string n = core::headerNeighbours(3);
    CHECK(has(n, "pin 1 (3V3)") && has(n, "pin 4 (5V)") && has(n, "pin 5 (GPIO3)"));
}

TEST(pcb_pull_verdicts) {
    const core::BoardLine& none = *core::boardLine(4);
    const core::BoardLine& up = *core::boardLine(10);
    const core::BoardLine& ms1 = *core::boardLine(20);
    const core::BoardLine& ms2 = *core::boardLine(21);
    CHECK(core::judgePull(none, 1, 0).level == Level::Pass);
    CHECK(core::judgePull(none, 1, 1).level == Level::Fail && has(core::judgePull(none, 1, 1).text, "HIGH"));
    CHECK(core::judgePull(none, 0, 0).level == Level::Fail && has(core::judgePull(none, 0, 0).text, "LOW"));
    CHECK(core::judgePull(none, 0, 1).level == Level::Fail);
    CHECK(core::judgePull(up, 1, 1).level == Level::Pass);
    CHECK(core::judgePull(up, 1, 0).level == Level::Fail && has(core::judgePull(up, 1, 0).text, "missing"));
    CHECK(core::judgePull(up, 0, 0).level == Level::Fail);
    CHECK(core::judgePull(ms1, 1, 0).level == Level::Pass);
    CHECK(core::judgePull(ms1, 0, 0).level == Level::Warn);
    CHECK(core::judgePull(ms1, 1, 1).level == Level::Fail);
    CHECK(core::judgePull(ms2, 1, 0).level == Level::Pass);
    CHECK(core::judgePull(ms2, 0, 0).level == Level::Pass);   // the pull-up reading is undefined there
    CHECK(core::judgePull(ms2, 1, 1).level == Level::Fail);
    // the neighbour hint names the 5 V pin next to GPIO2
    CHECK(has(core::judgePull(*core::boardLine(2), 0, 0).text, "pin 4 (5V)"));
}

TEST(pcb_bridges_are_found_and_unwitnessable_lines_ignored) {
    // lines: 4 (none), 17 (none), 2 (pulled up), 21 (MS2, 50k pull-down)
    const std::vector<int> bcm = {4, 17, 2, 21};
    const std::vector<int> up = {1, 1, 1, 0}, down = {0, 0, 1, 0};
    // 4 and 17 bridged: each sees the other in both tests. 4 and 2 bridged: only the low test can show it (2 is high
    // under either pull). 21 reads 0 under the pull-up anyway, so it witnesses nothing in the low test.
    std::vector<std::vector<int>> high = {{-1, 1, 1, 0}, {1, -1, 1, 0}, {0, 0, -1, 0}, {0, 0, 1, -1}};
    std::vector<std::vector<int>> low = {{-1, 0, 0, 0}, {0, -1, 1, 0}, {0, 1, -1, 0}, {1, 1, 1, -1}};
    const auto b = core::findBridges(bcm, up, down, high, low);
    CHECK_EQ(b.size(), static_cast<std::size_t>(2));
    CHECK(b[0].a == 2 && b[0].b == 4 && !b[0].high && b[0].low);
    CHECK(b[1].a == 4 && b[1].b == 17 && b[1].high && b[1].low);
    // nothing bridged: no findings
    std::vector<std::vector<int>> h0 = {{-1, 0, 1, 0}, {0, -1, 1, 0}, {0, 0, -1, 0}, {0, 0, 1, -1}};
    std::vector<std::vector<int>> l0 = {{-1, 1, 1, 0}, {1, -1, 1, 0}, {1, 1, -1, 0}, {1, 1, 1, -1}};
    CHECK(core::findBridges(bcm, up, down, h0, l0).empty());
}

TEST(pcb_encoder_turn_analysis) {
    const double even[4] = {1, 1, 1, 1};
    const auto e = turn(5, 2400, -1, even);
    const core::EncoderTurn t = core::analyzeTurn(5, e);
    CHECK_EQ(t.net, -2400L);
    CHECK_EQ(t.edges, 2400L);
    CHECK_EQ(t.chanEven, 1200L);
    CHECK_EQ(t.chanOdd, 1200L);
    CHECK_EQ(t.reversals, 0L);
    CHECK_EQ(t.jumps, 0L);
    CHECK(t.cycles >= 590);
    for (double q : t.quarter) CHECK_NEAR(q, 0.25, 1e-9);
    CHECK(core::judgeTurn(t, -2400, 6).level == Level::Pass);
    // the same turn counted the other way: A and B swapped
    const core::EncoderTurn r = core::analyzeTurn(5, turn(5, 2400, +1, even));
    const core::Judgement jr = core::judgeTurn(r, -2400, 6);
    CHECK(jr.level == Level::Fail && has(jr.text, "OTHER way"));
    // one dead channel: the count only flips between two values
    std::vector<core::EdgeSample> dead;
    for (int i = 0; i < 600; ++i) dead.push_back({1000000LL * (i + 1), i % 2 ? 0 : 1});
    const core::Judgement jd = core::judgeTurn(core::analyzeTurn(0, dead), -2400, 6);
    CHECK(jd.level == Level::Fail && has(jd.text, "one channel"));
    // no edges at all
    CHECK(core::judgeTurn(core::analyzeTurn(0, {}), -2400, 6).level == Level::Fail);
    // a duty / phase error shows in the quarters, both turns together
    const double skew[4] = {0.6, 1.4, 0.6, 1.4};
    const core::EncoderTurn s1 = core::analyzeTurn(0, turn(0, 2400, -1, skew));
    const core::EncoderTurn s2 = core::analyzeTurn(-2400, turn(-2400, 2400, +1, skew));
    CHECK_NEAR(s1.quarter[0], 0.15, 1e-6);
    CHECK_NEAR(s1.quarter[1], 0.35, 1e-6);
    CHECK(core::judgeQuadrature(s1, s2).level == Level::Warn);
    CHECK(core::judgeQuadrature(t, t).level == Level::Pass);
    // a reversal splits the cycles but not the count
    std::vector<core::EdgeSample> back = turn(0, 40, -1, even);
    const auto fwd = turn(-40, 40, +1, even);
    for (auto x : fwd) { x.ts_ns += back.back().ts_ns; back.push_back(x); }
    const core::EncoderTurn tb = core::analyzeTurn(0, back);
    CHECK_EQ(tb.net, 0L);
    CHECK_EQ(tb.reversals, 1L);
}

TEST(pcb_step_mode_judgement) {
    // 1/8 reference: 200 eighth-steps out, 267 back (switch hysteresis + back-off + debounce = 67)
    std::vector<core::StepTrial> ok = {{"1/8", 1, 200, 267}, {"1/4", 2, 100, 268}, {"1/2", 4, 50, 266},
                                       {"full", 8, 25, 267}, {"1/8 again", 1, 200, 267}};
    for (const auto& j : core::judgeStepModes(ok, 0.10)) CHECK(j.level == Level::Pass);
    // MS1 and MS2 swapped: "1/4" steps like 1/2 (twice as far), "1/2" like 1/4 (half as far)
    std::vector<core::StepTrial> swapped = {{"1/8", 1, 200, 267}, {"1/4", 2, 100, 467}, {"1/2", 4, 50, 167}};
    const auto js = core::judgeStepModes(swapped, 0.10);
    CHECK(js[1].level == Level::Fail && has(js[1].text, "like 1/2"));
    CHECK(js[2].level == Level::Fail && has(js[2].text, "like 1/4"));
    // switch not found on the way back
    std::vector<core::StepTrial> lost = {{"1/8", 1, 200, 267}, {"1/4", 2, 100, -1}};
    CHECK(core::judgeStepModes(lost, 0.10)[1].level == Level::Fail);
}

TEST(pcb_supply_and_range_verdicts) {
    CHECK(core::judgeThrottled(0, "x").level == Level::Pass);
    CHECK(core::judgeThrottled(0x50000, "x").level == Level::Fail);   // under-voltage + throttling occurred
    CHECK(core::judgeThrottled(0x1, "x").level == Level::Fail);
    CHECK(core::judgeThrottled(0x80008, "x").level == Level::Warn);   // soft temperature limit
    CHECK(core::judgeRange("5V", 5.02, 4.90, 5.25, "V").level == Level::Pass);
    CHECK(core::judgeRange("5V", 5.45, 4.90, 5.25, "V").level == Level::Fail);
}
