#ifndef PENDEL_CORE_PCB_CHECK_H
#define PENDEL_CORE_PCB_CHECK_H

#include <cstdint>
#include <string>
#include <vector>

// Bring-up checks of the carrier PCB (pendctl --mode pcb-test, docs/pcb_bringup.md): the board's GPIO wiring as
// designed (pcb/PCB.kicad_pcb, 2026-10-06) and the judgements, pure and unit-tested on the PC
// (tests/unit/test_pcb_check.cpp). The Pi's internal pulls are ~50-65 kOhm.
namespace core {

enum class Level { Pass = 0, Info = 1, Warn = 2, Fail = 3, Skip = 4 };
const char* levelName(Level l);

// What the board (and the Pi itself) puts on a GPIO line, as the internal pull sees it.
enum class LineLoad {
    None,          // nothing: follows the internal pull (1 with pull-up, 0 with pull-down)
    PullUp,        // <= 10 kOhm to 3V3 (board resistor, or the Pi's 1.8 kOhm on GPIO2/3): 1 with either pull
    WeakPullDown,  // A4988 MS1, 100 kOhm to GND: ~2.2 V with the internal pull-up (reads 1, close to the threshold)
    PullDown50k,   // A4988 MS2/MS3, 50 kOhm to GND: ~1.65 V with the internal pull-up (undefined), 0 with pull-down
};

struct BoardLine {
    int bcm;
    int header;            // physical pin on the 40-pin header
    const char* signal;    // what the board connects
    LineLoad load;
    bool output;           // the Pi drives it in operation (A4988 inputs)
};

const std::vector<BoardLine>& boardLines();            // GPIO 0-27, in BCM order
const BoardLine* boardLine(int bcm);
std::string headerPinName(int header);                 // "3V3", "5V", "GND" or "GPIOn"
std::string headerNeighbours(int header);              // the pins a solder bridge could reach

struct Judgement {
    Level level;
    std::string text;
};

// Line read once with the internal pull-up, once with the pull-down (nothing else driving it).
Judgement judgePull(const BoardLine& l, int readPullUp, int readPullDown);

// Bridge test. Drivers are driven high with every other line pulled down, then low with every other line pulled
// up. A line can witness a bridge only if its own reading under that pull showed the opposite level
// (baselineDown == 0 for the high test, baselineUp == 1 and not PullDown50k/WeakPullDown for the low test).
// high[i][j] / low[i][j]: reading of line j while line i was driven (-1: not read / not driven).
struct Bridge {
    int a, b;              // BCM numbers, a < b
    bool high, low;        // seen in the high / low test
};
std::vector<Bridge> findBridges(const std::vector<int>& bcm, const std::vector<int>& baselineUp,
                                const std::vector<int>& baselineDown, const std::vector<std::vector<int>>& high,
                                const std::vector<std::vector<int>>& low);

// Encoder: edges as hw::EdgeReader reports them (timestamp, count after the edge).
struct EdgeSample {
    std::int64_t ts_ns;
    std::int64_t count;
};
struct EncoderTurn {
    long net = 0;              // last count - count before the first edge
    long edges = 0;
    long chanEven = 0;         // edges between counts 2k and 2k+1: one channel ...
    long chanOdd = 0;          // ... and between 2k+1 and 2k+2: the other
    long jumps = 0;            // steps of more than one count (should never happen)
    long reversals = 0;        // direction changes
    double quarter[4] = {};    // share of a quadrature cycle per quarter (by lower count mod 4), ideal 0.25 each
    long cycles = 0;           // complete monotonic cycles that went into quarter[]
};
EncoderTurn analyzeTurn(std::int64_t count0, const std::vector<EdgeSample>& edges);
// One deliberate turn: expectNet = +-cpr (sign = expected direction); tolerance in counts.
Judgement judgeTurn(const EncoderTurn& t, long expectNet, long tolerance);
Judgement judgeQuadrature(const EncoderTurn& a, const EncoderTurn& b);   // duty / phase, from both turns

// Step modes against the limit switch: each trial moves away by `pulses` in a mode of `usteps_per_pulse` (1/8 = 1,
// 1/4 = 2, 1/2 = 4, full = 8 eighth-steps per pulse) and comes back in 1/8 counting eighth-steps to the switch.
// The first trial (1/8) is the reference: distance(trial) = return(trial) - return(first) + expected(first).
struct StepTrial {
    const char* name;
    int usteps_per_pulse;
    long pulses;
    long returned;             // eighth-steps back to the switch; < 0: not found
};
std::vector<Judgement> judgeStepModes(const std::vector<StepTrial>& trials, double tolerance);

// Raspberry Pi `vcgencmd get_throttled` bits.
Judgement judgeThrottled(unsigned bits, const char* when);
Judgement judgeRange(const std::string& what, double v, double lo, double hi, const char* unit);

}

#endif
