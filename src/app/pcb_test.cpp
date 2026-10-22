// pcb-test: bring-up test of the carrier PCB (pcb/, docs/pcb_bringup.md), one stage per run (pcbtest.stage):
//   C  the board on the Pi, J3/J4/J9 empty, motor fuse F2 out. Every header GPIO: shorts to the rails (read with
//      each internal pull), solder bridges to the others (each line driven ~0.3 ms high, then low, while the rest
//      watch), drive readback. Supply: get_throttled before and after 30 s of full CPU load; 5V, 3V3 and the
//      A4988's Vref measured by hand and entered.
//   D  limit switches in J3, encoder in J4 (F2 still out): each switch pressed on its own; encoder still for 10 s,
//      then one turn each way (counts, direction, both channels, glitches, quadrature timing).
//   E  MOTOR MOVES (motor in J9, F2 in, stage D passed): driver hold and release, direction, the four step modes
//      measured against the right limit switch, switch-to-switch span, back to the middle. At most 20 mm/s.
// HARDWARE ACTION: opens the real chip, refused unless pcbtest.allow_real_chip (./pcbtest.sh sets it after its
// checklist). Only stage E energises the motor, and only after a go typed for each move. On every exit: ~ENABLE
// high (driver off), MS1/MS2 high (1/8 step), STEP and DIR low; every other line it touched an input with its
// power-on pull (GPIO0-8 up, 9-27 down). Ctrl-C stops a move at once (driver off) and still writes the report.

#include <gpiod.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "app/modes.h"
#include "common/runlog.h"
#include "core/pcb_check.h"
#include "hw/edge_reader.h"
#include "rt/rt.h"

namespace app {
namespace {

using core::Judgement;
using core::Level;

const char* kConsumer = "pcb-test";
// The board's wiring (pcb/PCB.kicad_pcb); config pins.* must agree.
constexpr int kStep = 27, kDir = 22, kEn = 24, kMs1 = 20, kMs2 = 21, kSwR = 2, kSwL = 9, kEncA = 10, kEncB = 19;

struct Settings {
    std::string stage, logDir;
    double loadSeconds, vSlow, vSpan, seekMax, dirTest, backoff, holdSeconds, upm;
    long trialUsteps, cpr, spanUsteps;
    bool requireD;
};

// ---------------------------------------------------------------- report

struct Item {
    std::string id, name;
    Level level;
    std::string detail;
};

class Report {
public:
    void add(const std::string& id, const std::string& name, Level l, const std::string& d) {
        items_.push_back({id, name, l, d});
        std::printf("  %-4s %-4s %s: %s\n", id.c_str(), core::levelName(l), name.c_str(), d.c_str());
        std::fflush(stdout);
    }
    void add(const std::string& id, const std::string& name, const Judgement& j) { add(id, name, j.level, j.text); }
    int count(Level l) const {
        return static_cast<int>(std::count_if(items_.begin(), items_.end(), [l](const Item& i) { return i.level == l; }));
    }
    std::string result(bool completed) const {
        if (count(Level::Fail)) return "FAIL";
        if (!completed) return "INCOMPLETE";
        return count(Level::Warn) ? "PASS with warnings" : "PASS";
    }
    void write(const std::string& path, const std::string& head) const {
        std::ofstream f(path);
        f << head << "\n\n";
        for (const Item& i : items_) f << i.id << "\t" << core::levelName(i.level) << "\t" << i.name << "\t" << i.detail << "\n";
    }
    void summary() const {
        std::printf("\n== %d PASS, %d WARN, %d FAIL, %d SKIP\n", count(Level::Pass), count(Level::Warn), count(Level::Fail),
                    count(Level::Skip));
        for (const Item& i : items_)
            if (i.level == Level::Fail || i.level == Level::Warn)
                std::printf("  %-4s %s %s: %s\n", core::levelName(i.level), i.id.c_str(), i.name.c_str(), i.detail.c_str());
    }

private:
    std::vector<Item> items_;
};

// ---------------------------------------------------------------- operator input (Ctrl-C aware)

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

// One line typed by the operator; false on Ctrl-C, EOF or timeout (timeout < 0: none).
bool readLine(std::string* out, double timeout = -1) {
    out->clear();
    const std::int64_t end = timeout < 0 ? INT64_MAX : rt::monoNs() + static_cast<std::int64_t>(timeout * 1e9);
    std::string line;
    while (!flags.stop && rt::monoNs() < end) {
        pollfd p{STDIN_FILENO, POLLIN, 0};
        if (poll(&p, 1, 100) <= 0) continue;
        char c;
        if (read(STDIN_FILENO, &c, 1) <= 0) return false;
        if (c == '\n') {
            *out = trim(line);
            return true;
        }
        if (c != '\r') line += c;
    }
    return false;
}

bool askYes(const std::string& q) {
    std::printf("\n%s\n  Type yes to continue: ", q.c_str());
    std::fflush(stdout);
    std::string s;
    return readLine(&s) && s == "yes";
}

// 'y', 'n', 's' (skip) or 'q' (stop / Ctrl-C).
char askYN(const std::string& q) {
    for (;;) {
        std::printf("  %s [y/n, s = skip, q = stop]: ", q.c_str());
        std::fflush(stdout);
        std::string s;
        if (!readLine(&s)) return 'q';
        if (s.empty()) continue;
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(s[0])));
        if (c == 'y' || c == 'n' || c == 's' || c == 'q') return c;
    }
}

bool waitEnter(const std::string& q) {
    std::printf("\n%s\n  Press Enter to start (q + Enter = stop): ", q.c_str());
    std::fflush(stdout);
    std::string s;
    return readLine(&s) && s != "q";
}

std::optional<double> askNumber(const std::string& q) {
    for (;;) {
        std::printf("  %s (Enter = skip): ", q.c_str());
        std::fflush(stdout);
        std::string s;
        if (!readLine(&s) || s.empty()) return std::nullopt;
        char* end = nullptr;
        const double v = std::strtod(s.c_str(), &end);
        if (end && *end == 0) return v;
        std::printf("  (a number please, e.g. 5.02)\n");
    }
}

void sleepMs(double ms) { rt::sleepUntilNs(rt::monoNs() + static_cast<std::int64_t>(ms * 1e6)); }

std::string shell(const std::string& cmd) {
    std::string out;
    if (FILE* p = popen(cmd.c_str(), "r")) {
        char b[256];
        while (std::fgets(b, sizeof b, p)) out += b;
        pclose(p);
    }
    return out;
}

// ---------------------------------------------------------------- GPIO

class Gpio {
public:
    explicit Gpio(gpiod_chip* c) : chip_(c) {}
    ~Gpio() {
        for (auto& m : mode_)
            if (m.second) gpiod_line_release(line(m.first));
    }
    gpiod_line* line(int bcm) { return gpiod_chip_get_line(chip_, static_cast<unsigned>(bcm)); }
    bool claimedElsewhere(int bcm) {
        gpiod_line* l = line(bcm);
        return !l || (mode_[bcm] == 0 && gpiod_line_is_used(l));
    }
    // bias: +1 pull-up, -1 pull-down, 0 none
    bool input(int bcm, int bias) {
        drop(bcm);
        const int f = bias > 0 ? GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
                               : bias < 0 ? GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN : GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
        if (gpiod_line_request_input_flags(line(bcm), kConsumer, f) != 0) return false;
        touched_.insert(bcm);
        mode_[bcm] = 1;
        return true;
    }
    bool output(int bcm, int v) {
        drop(bcm);
        if (gpiod_line_request_output(line(bcm), kConsumer, v) != 0) return false;
        touched_.insert(bcm);
        mode_[bcm] = 2;
        return true;
    }
    int get(int bcm) { return gpiod_line_get_value(line(bcm)); }   // pad level, also for outputs
    void set(int bcm, int v) { gpiod_line_set_value(line(bcm), v); }
    void drop(int bcm) {
        if (mode_[bcm]) {
            gpiod_line_release(line(bcm));
            mode_[bcm] = 0;
        }
    }
    const std::set<int>& touched() const { return touched_; }

private:
    gpiod_chip* chip_;
    std::map<int, int> mode_;   // 0 free, 1 input, 2 output
    std::set<int> touched_;
};

// The rig's safe state; outputs keep their level after release (persist_gpio_outputs=Y).
void restoreSafe(Gpio& g) {
    g.output(kEn, 1);
    flags.motionSafe = true;
    activeEnLine = nullptr;
    const std::set<int> touched = g.touched();
    for (int b : touched) {
        if (b == kEn) continue;
        if (b == kMs1 || b == kMs2) g.output(b, 1);
        else if (b == kStep || b == kDir) g.output(b, 0);
        else g.input(b, b <= 8 ? +1 : -1);
        g.drop(b);
    }
    g.drop(kEn);
}

struct PinState {
    std::string func, level, raw;
};

// `pinctrl get N`, e.g. " 9: ip    pd | lo // GPIO9 = input", "24: op dh pd | hi // GPIO24 = output"
PinState pinctrlGet(int bcm) {
    PinState s;
    const std::string out = shell("pinctrl get " + std::to_string(bcm) + " 2>/dev/null");
    s.raw = trim(out);
    const std::size_t colon = out.find(':'), bar = out.find('|');
    if (colon == std::string::npos || bar == std::string::npos || bar < colon) return s;
    std::istringstream is(out.substr(colon + 1, bar - colon - 1));
    is >> s.func;
    std::istringstream il(out.substr(bar + 1));
    il >> s.level;
    return s;
}

std::string pinLabel(int bcm) {
    const core::BoardLine* l = core::boardLine(bcm);
    return "GPIO" + std::to_string(bcm) + (l ? " (pin " + std::to_string(l->header) + ")" : "");
}

// ---------------------------------------------------------------- stage C

bool stageC(Gpio& g, bool mock, Report& r, const Settings& st) {
    if (!askYes("STAGE C: the board is on the Pi and powered through it (no USB-C cable), J3 (switches), J4 (encoder) "
                "and J9 (motor) are EMPTY, the motor fuse F2 is REMOVED. Stages A and B of docs/pcb_bringup.md passed."))
        return false;

    std::printf("\nC1  which lines can be tested\n");
    std::vector<int> lines;
    for (const core::BoardLine& l : core::boardLines()) {
        if (!mock) {
            const PinState ps = pinctrlGet(l.bcm);
            if (l.bcm == kEn) {
                if (ps.func == "op" && ps.level == "hi")
                    r.add("C1", "driver off at boot", Level::Pass, "GPIO24 = output high since boot (config.txt gpio=24=op,dh)");
                else
                    r.add("C1", "driver off at boot", Level::Warn,
                          "GPIO24 is '" + ps.raw + "', not output-high: the driver may be enabled whenever no program holds it. "
                          "Check /boot/firmware/config.txt for gpio=24=op,dh");
            }
            if (ps.func != "ip" && ps.func != "op") {
                r.add("C1", pinLabel(l.bcm), Level::Skip,
                      "in use as '" + (ps.func.empty() ? ps.raw : ps.func) + "' (an alternate function): not touched");
                continue;
            }
        }
        if (g.claimedElsewhere(l.bcm)) {
            r.add("C1", pinLabel(l.bcm), Level::Skip, "claimed by a driver or another program: not touched");
            continue;
        }
        lines.push_back(l.bcm);
    }
    std::printf("  %zu of 28 header GPIOs will be tested\n", lines.size());

    std::printf("\nC2  shorts: each line read with the internal pull-up, then the pull-down\n");
    const std::size_t n = lines.size();
    std::vector<int> up(n, -1), dn(n, -1);
    std::vector<bool> drivable(n, false);
    for (std::size_t i = 0; i < n && !flags.stop; ++i) {
        const int b = lines[i];
        if (!g.input(b, +1)) { r.add("C2", pinLabel(b), Level::Fail, "could not be requested"); continue; }
        sleepMs(2);
        up[i] = g.get(b);
        g.input(b, -1);
        sleepMs(2);
        dn[i] = g.get(b);
        const Judgement j = core::judgePull(*core::boardLine(b), up[i], dn[i]);
        r.add("C2", pinLabel(b) + " " + core::boardLine(b)->signal, j);
        drivable[i] = j.level != Level::Fail;   // never drive a line that is held by something: that is a short
    }
    if (flags.stop) return false;

    std::printf("\nC3  bridges: each line driven high (others pulled down), then low (others pulled up), ~0.3 ms each\n");
    std::vector<std::vector<int>> high(n, std::vector<int>(n, -1)), low(n, std::vector<int>(n, -1));
    std::vector<int> selfHigh(n, -1), selfLow(n, -1);
    for (int phase = 0; phase < 2 && !flags.stop; ++phase) {
        const int bias = phase == 0 ? -1 : +1, drive = phase == 0 ? 1 : 0;
        for (std::size_t i = 0; i < n; ++i) g.input(lines[i], bias);
        sleepMs(3);
        for (std::size_t i = 0; i < n && !flags.stop; ++i) {
            if (!drivable[i]) continue;
            if (!g.output(lines[i], drive)) continue;
            rt::sleepUntilNs(rt::monoNs() + 300000);
            (phase == 0 ? selfHigh : selfLow)[i] = g.get(lines[i]);
            for (std::size_t j = 0; j < n; ++j)
                if (j != i) (phase == 0 ? high : low)[i][j] = g.get(lines[j]);
            g.input(lines[i], bias);
            sleepMs(1);
        }
    }
    if (flags.stop) return false;
    const std::vector<core::Bridge> br = core::findBridges(lines, up, dn, high, low);
    for (const core::Bridge& b : br)
        r.add("C3", "bridge", Level::Fail,
              pinLabel(b.a) + " and " + pinLabel(b.b) + " are connected (seen driving " +
                  (b.high && b.low ? "high and low" : b.high ? "high" : "low") +
                  "): a solder bridge or a short. Look between the two header pins and along both traces");
    if (br.empty())
        r.add("C3", "bridges", Level::Pass, "none between the " + std::to_string(std::count(drivable.begin(), drivable.end(), true)) +
                                                " drivable lines (each driven high and low while the others watched)");
    for (std::size_t i = 0; i < n; ++i) {
        const core::BoardLine* l = core::boardLine(lines[i]);
        if (!drivable[i]) continue;
        if (selfHigh[i] != 1 || selfLow[i] != 0)
            r.add("C3", pinLabel(lines[i]) + " drive", Level::Fail,
                  "driven high reads " + std::to_string(selfHigh[i]) + ", driven low reads " + std::to_string(selfLow[i]) +
                      ": the pad can't reach its level, something loads it hard");
        else if (l && l->output)
            r.add("C3", pinLabel(lines[i]) + " drive", Level::Pass, std::string(l->signal) + ": drives high and low");
    }
    restoreSafe(g);

    std::printf("\nC4  supply under load\n");
    if (mock) {
        r.add("C4", "supply", Level::Skip, "mock chip");
    } else {
        auto throttled = [](unsigned* v) {
            const std::string s = shell("vcgencmd get_throttled 2>/dev/null");
            const std::size_t p = s.find("0x");
            if (p == std::string::npos) return false;
            *v = static_cast<unsigned>(std::strtoul(s.c_str() + p, nullptr, 16));
            return true;
        };
        unsigned v0 = 0, v1 = 0;
        if (throttled(&v0)) r.add("C4", "since boot", core::judgeThrottled(v0, "since boot"));
        else r.add("C4", "since boot", Level::Skip, "vcgencmd get_throttled not available");
        if (waitEnter("All four CPU cores now run flat out for " + std::to_string(static_cast<int>(st.loadSeconds)) +
                      " s (the Pi's highest current). Meanwhile measure with the multimeter, black lead clipped to J2's "
                      "GND terminal: 5V at C4's + leg (the leg without the stripe), 3V3 at R3's 3V3 end (the end that "
                      "beeps to header pin 1 with the power off). NEVER probe the header pins while the Pi is on: a slip "
                      "between pin 1 and pin 2 puts 5 V on the Pi's 3.3 V rail and destroys the Pi.")) {
            std::atomic<bool> run{true};
            std::vector<std::thread> burn;
            for (int k = 0; k < 4; ++k)
                burn.emplace_back([&run] {
                    volatile double x = 1.0;
                    while (run.load(std::memory_order_relaxed)) x = x * 1.0000001 + 1e-9;
                });
            const std::int64_t end = rt::monoNs() + static_cast<std::int64_t>(st.loadSeconds * 1e9);
            int dots = 0;
            while (rt::monoNs() < end && !flags.stop) {
                sleepMs(1000);
                std::printf("%s", ++dots % 10 ? "." : "|");
                std::fflush(stdout);
            }
            run = false;
            for (auto& t : burn) t.join();
            std::printf("\n");
            if (throttled(&v1)) r.add("C4", "after the load", core::judgeThrottled(v1, "after the load"));
            const std::string temp = trim(shell("vcgencmd measure_temp 2>/dev/null"));
            if (!temp.empty()) r.add("C4", "SoC temperature", Level::Info, temp);
        }
        if (auto v = askNumber("5V rail you measured under load, in V")) r.add("C4", "5V rail", core::judgeRange("5V (C4 +)", *v, 4.90, 5.25, "V"));
        else r.add("C4", "5V rail", Level::Skip, "not measured");
        if (auto v = askNumber("3V3 rail, in V")) r.add("C4", "3V3 rail", core::judgeRange("3V3 (R3)", *v, 3.20, 3.40, "V"));
        else r.add("C4", "3V3 rail", Level::Skip, "not measured");
    }

    std::printf("\nC5  A4988 current limit\n");
    std::printf("  With the Pi on, the A4988's logic is powered (VDD = 3V3). Measure Vref: black lead clipped to J2's GND,\n"
                "  red probe on the potentiometer's metal top only (not the header or the carrier's pins). Don't turn it.\n");
    if (auto v = askNumber("Vref, in V")) {
        char s[240];
        std::snprintf(s, sizeof s,
                      "Vref %.3f V -> current limit %.2f A with 0.068 ohm sense resistors (Pololu since 2017, 'R068'), "
                      "%.2f A with 0.050 ('R050'), %.2f A with 0.100 ('R100', most clones). Record it.",
                      *v, *v / (8 * 0.068), *v / (8 * 0.050), *v / (8 * 0.100));
        r.add("C5", "Vref", *v > 0.05 ? Level::Info : Level::Warn,
              std::string(s) + (*v > 0.05 ? "" : " ~0 V: no current; check the carrier is seated and its VDD pad has 3.3 V"));
    } else {
        r.add("C5", "Vref", Level::Skip, "not measured");
    }
    return true;
}

// ---------------------------------------------------------------- stage D

// Operator presses and releases `target`; `other` must not react.
void switchTest(Gpio& g, Report& r, int target, int other, const char* name) {
    std::printf("\n  Press and HOLD the %s limit switch for about a second, then let go. 20 s (s + Enter = skip).\n", name);
    std::fflush(stdout);
    const std::int64_t end = rt::monoNs() + 20000000000LL;
    int last = 1, lastOther = 1, changes = 0, otherChanges = 0;
    bool pressed = false, released = false, otherReleased = false;
    std::int64_t tRelease = 0;
    while (rt::monoNs() < end && !flags.stop) {
        pollfd p{STDIN_FILENO, POLLIN, 0};
        if (poll(&p, 1, 1) > 0) {
            std::string s;
            if (readLine(&s, 0.5) && s == "s") break;
        }
        const int v = g.get(target), o = g.get(other);
        if (v != last) {
            ++changes;
            last = v;
            if (v == 0) pressed = true;
            if (v == 1 && pressed) { released = true; tRelease = rt::monoNs(); }
        }
        if (o != lastOther) {
            ++otherChanges;
            lastOther = o;
            if (o == 1) { otherReleased = true; tRelease = rt::monoNs(); }
        }
        // a full press on either line ends the window (on the wrong line: swapped wires, not "no reaction" 20 s later)
        if ((released || otherReleased) && last == 1 && lastOther == 1 && rt::monoNs() - tRelease > 300000000LL) break;
    }
    const std::string label = std::string(name) + " switch (" + pinLabel(target) + ")";
    if (pressed && released && otherChanges == 0)
        r.add("D2", label, Level::Pass,
              "0 while pressed, 1 released; " + std::to_string(std::max(0, changes - 2)) + " contact bounces; the other switch stayed still");
    else if (!pressed && otherChanges > 0)
        r.add("D2", label, Level::Fail, "the OTHER switch's line reacted instead: the switches are swapped on J3 (pins 2 and 3) "
                                        "or the wires are crossed. Homing would drive into the wrong end");
    else if (pressed && !released)
        r.add("D2", label, Level::Fail, "went to 0 and never back to 1: the switch sticks, or it is wired normally-closed");
    else if (pressed && otherChanges > 0)
        r.add("D2", label, Level::Fail, "both switch lines reacted: they are connected (a bridge on J3 or in the cable)");
    else
        r.add("D2", label, Level::Fail, "no reaction: switch not connected to J3 pin " + std::string(target == kSwR ? "2" : "3") +
                                            ", a broken wire, or the switch itself");
}

struct Recorded {
    std::int64_t count0 = 0;
    std::vector<core::EdgeSample> edges;
};

// Edges until Enter (or `seconds` when > 0, no Enter needed).
Recorded record(hw::EdgeReader& er, double seconds, double maxSeconds) {
    Recorded rec;
    rec.count0 = er.counts();
    hw::Edge buf[256];
    bool ovf = false;
    er.drainEdges(buf, 256, &ovf);   // discard anything older
    rec.count0 = er.counts();
    const std::int64_t t0 = rt::monoNs();
    const std::int64_t end = t0 + static_cast<std::int64_t>((seconds > 0 ? seconds : maxSeconds) * 1e9);
    while (rt::monoNs() < end && !flags.stop) {
        const int k = er.drainEdges(buf, 256, &ovf);
        for (int i = 0; i < k; ++i) rec.edges.push_back({buf[i].ts_ns, buf[i].count_after});
        if (seconds <= 0) {
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (poll(&p, 1, 5) > 0) {
                std::string s;
                if (readLine(&s, 0.5)) break;
            }
        } else {
            sleepMs(5);
        }
    }
    const int k = er.drainEdges(buf, 256, &ovf);
    for (int i = 0; i < k; ++i) rec.edges.push_back({buf[i].ts_ns, buf[i].count_after});
    return rec;
}

bool stageD(gpiod_chip* chip, Gpio& g, Report& r, const Settings& st, const std::string& dir) {
    if (!askYes("STAGE D: with the power OFF you plugged the limit switches into J3 and the encoder into J4. F2 still "
                "REMOVED, no motor in J9. Stage C passed."))
        return false;

    std::printf("\nD1  switch lines at rest\n");
    g.input(kSwR, +1);
    g.input(kSwL, +1);
    int minR = 1, minL = 1;
    for (int k = 0; k < 500; ++k) {
        minR = std::min(minR, g.get(kSwR));
        minL = std::min(minL, g.get(kSwL));
        sleepMs(1);
    }
    r.add("D1", "switches at rest", minR == 1 && minL == 1 ? Level::Pass : Level::Fail,
          minR == 1 && minL == 1 ? "both read 1 (open, normally-open switches)"
                                 : std::string("reads 0 at rest:") + (minR ? "" : " RIGHT") + (minL ? "" : " LEFT") +
                                       ". Pressed by the cart, wired normally-closed, or shorted to GND");
    if (minR != 1 || minL != 1) return true;

    std::printf("\nD2  switches one at a time\n");
    switchTest(g, r, kSwR, kSwL, "RIGHT (at the low-position end)");
    if (flags.stop) return false;
    switchTest(g, r, kSwL, kSwR, "LEFT");
    g.drop(kSwR);
    g.drop(kSwL);
    if (flags.stop) return false;

    std::printf("\nD3  encoder\n");
    hw::EdgeReader er(chip, kEncA, kEncB);
    std::string err;
    if (!er.start(0, -1, &err)) {
        r.add("D3", "encoder", Level::Fail, "the encoder lines could not be opened: " + err);
        return true;
    }
    std::printf("  Let the pendulum hang still, hands off, for 10 s.\n");
    std::fflush(stdout);
    sleepMs(1500);
    const Recorded rest = record(er, 10.0, 10.0);
    const long nRest = static_cast<long>(rest.edges.size());
    r.add("D3", "encoder at rest", nRest <= 4 ? Level::Pass : nRest <= 20 ? Level::Warn : Level::Fail,
          std::to_string(nRest) + " edges in 10 s" +
              (nRest <= 4 ? " (a resting disc may sit on an edge and flicker once or twice)"
                          : ": noise on the encoder lines (cable near the motor wires? the shield? the 5 V supply?)"));
    if (flags.stop) return false;

    std::printf("\n  Now one slow turn: with the pendulum hanging, push the bob toward the LEFT switch (+x) and keep it going\n"
                "  the same way, over the top, all the way round once until it hangs again. Let it settle, then press Enter.\n");
    std::fflush(stdout);
    const Recorded t1 = record(er, -1, 90.0);
    const core::EncoderTurn a = core::analyzeTurn(t1.count0, t1.edges);
    r.add("D3", "turn toward +x", core::judgeTurn(a, -st.cpr, 6));
    if (flags.stop) return false;

    std::printf("\n  And back: push the bob toward the RIGHT switch and turn it once the other way, let it settle, Enter.\n");
    std::fflush(stdout);
    const Recorded t2 = record(er, -1, 90.0);
    const core::EncoderTurn b = core::analyzeTurn(t2.count0, t2.edges);
    r.add("D3", "turn toward -x", core::judgeTurn(b, +st.cpr, 6));
    r.add("D3", "quadrature timing", core::judgeQuadrature(a, b));
    const unsigned same = er.sameType(), full = er.fullBatches();
    r.add("D3", "decoder integrity", same == 0 && full == 0 ? Level::Pass : Level::Fail,
          std::to_string(same) + " impossible transitions, " + std::to_string(full) + " full kernel batches" +
              (same == 0 && full == 0 ? "" : ": glitches on A/B (noise, a marginal joint on R1/R2 or J4) or missed edges"));
    er.stop();

    std::ofstream f(dir + "/encoder_turns.csv");
    f << "turn,ts_ns,count\n";
    for (const auto& e : t1.edges) f << "1," << e.ts_ns << "," << e.count << "\n";
    for (const auto& e : t2.edges) f << "2," << e.ts_ns << "," << e.count << "\n";
    return true;
}

// ---------------------------------------------------------------- stage E

class Motor {
public:
    Motor(Gpio& g, double upm) : g_(g), upm_(upm) {}
    bool claim() {
        if (!g_.output(kEn, 1)) return false;
        activeEnLine = g_.line(kEn);
        return g_.output(kMs1, 1) && g_.output(kMs2, 1) && g_.output(kStep, 0) && g_.output(kDir, 0) && g_.input(kSwR, +1) &&
               g_.input(kSwL, +1);
    }
    void enable(bool on) {
        if (on) flags.motionSafe = false;
        g_.set(kEn, on ? 0 : 1);
        if (!on) flags.motionSafe = true;
    }
    // 1/8 = 1, 1/4 = 2, 1/2 = 4, full = 8 eighth-steps per pulse (A4988: MS1 MS2 = 11, 01, 10, 00; MS3 low)
    void mode(int upp) {
        g_.set(kMs1, upp == 1 || upp == 4 ? 1 : 0);
        g_.set(kMs2, upp == 1 || upp == 2 ? 1 : 0);
        sleepMs(2);
    }
    void dir(int d) {   // 1 = +x = toward the LEFT switch (Axis: DIR = 1 -> pos increasing)
        g_.set(kDir, d);
        sleepMs(2);
    }
    bool pressed(int sw) { return g_.get(sw) == 0; }
    double hz(double v) const { return v * upm_; }   // eighth-step pulses per s at v m/s

    // Up to n pulses at `rate` Hz; before each: Ctrl-C and `abortIf` stop it; after each: `doneIf` ends it.
    long pulses(long n, double rate, const std::function<bool()>& abortIf, const std::function<bool()>& doneIf,
                std::string* why) {
        const std::int64_t period = static_cast<std::int64_t>(1e9 / rate);
        std::int64_t next = rt::monoNs();
        long k = 0;
        for (; k < n; ++k) {
            if (flags.stop) { if (why) *why = "stopped (Ctrl-C)"; break; }
            if (abortIf && abortIf()) { if (why && why->empty()) *why = "aborted"; break; }
            g_.set(kStep, 1);
            const std::int64_t hi = rt::monoNs() + 5000;   // >= 1 us high for the A4988
            while (rt::monoNs() < hi) {}
            g_.set(kStep, 0);
            next += period;
            rt::sleepUntilNs(next);
            if (doneIf && doneIf()) { ++k; break; }
        }
        return k;
    }
    // Drive toward `sw` until it reads pressed 3 pulses in a row; -1 if not within maxPulses or aborted. `other`
    // (a switch that must stay open; -1 = none) closing aborts.
    long seek(int sw, int other, long maxPulses, double rate, std::string* why) {
        int stable = 0;
        bool found = false;
        const long k = pulses(
            maxPulses, rate,
            [&] {
                if (other >= 0 && pressed(other)) { *why = "the other switch closed"; return true; }
                return false;
            },
            [&] {
                stable = pressed(sw) ? stable + 1 : 0;
                found = stable >= 3;
                return found;
            },
            why);
        if (!found && why->empty()) *why = "switch not reached within " + std::to_string(maxPulses) + " eighth-steps";
        return found ? k : -1;
    }
    // Away from `sw` until it reads released 3 pulses in a row (max 20 mm), then `extra` pulses more.
    bool backOff(int sw, long extra, double rate, std::string* why) {
        int stable = 0;
        bool free = false;
        pulses(static_cast<long>(0.02 * upm_), rate, nullptr, [&] {
            stable = pressed(sw) ? 0 : stable + 1;
            free = stable >= 3;
            return free;
        }, why);
        if (!free) { if (why->empty()) *why = "the switch did not release within 20 mm"; return false; }
        pulses(extra, rate, nullptr, nullptr, why);
        return !flags.stop;
    }

private:
    Gpio& g_;
    double upm_;
};

std::string newestResult(const std::string& logDir, const std::string& suffix) {
    std::string best;
    if (DIR* d = opendir(logDir.c_str())) {
        while (dirent* e = readdir(d)) {
            const std::string n = e->d_name;
            if (n.size() > suffix.size() && n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0 && n > best) best = n;
        }
        closedir(d);
    }
    if (best.empty()) return "";
    std::ifstream f(logDir + "/" + best + "/meta.json");
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string m = ss.str();
    const std::size_t p = m.find("\"result\": \"");
    return p == std::string::npos ? "?" : best + ": " + m.substr(p + 11, m.find('"', p + 11) - p - 11);
}

bool stageE(Gpio& g, Report& r, const Settings& st) {
    const std::string d = newestResult(st.logDir, "_pcb-test-D");
    const bool dOk = d.find(": PASS") != std::string::npos;
    if (!dOk && st.requireD) {
        std::printf("pcb-test: stage E needs a passed stage D (the switches stop every move). Newest stage D: %s\n",
                    d.empty() ? "none" : d.c_str());
        r.add("E0", "stage D", Level::Fail, "no passed stage D in " + st.logDir + " (pcbtest.require_stage_d)");
        return false;
    }
    r.add("E0", "stage D", dOk ? Level::Pass : Level::Warn, d.empty() ? "none found (check overridden)" : d);
    if (!askYes("STAGE E, THE MOTOR WILL MOVE. With the power OFF you plugged the motor into J9 (never plug or unplug it "
                "with power on: that can destroy the A4988) and put F2 back in. Cart in the middle of the track, belt on, "
                "nothing on the track, pendulum hanging, hands clear of the belt and pulleys, motor power switch (rocker) "
                "within reach. Speeds here: at most 20 mm/s."))
        return false;

    Motor m(g, st.upm);
    if (!m.claim()) {
        r.add("E0", "lines", Level::Fail, "the motor lines could not be requested");
        return false;
    }
    if (m.pressed(kSwR) || m.pressed(kSwL)) {
        r.add("E0", "start position", Level::Fail, "a limit switch is pressed: push the cart off it by hand (driver is off)");
        return false;
    }
    const double slow = m.hz(st.vSlow), fast = m.hz(st.vSpan);
    std::string why;

    // E1 hold
    if (!waitEnter("E1  The driver is ENABLED for " + std::to_string(static_cast<int>(st.holdSeconds)) +
                   " s, with no steps. Meanwhile push the cart gently along the track by hand: it should resist (the "
                   "motor holds). Then it releases."))
        return false;
    m.enable(true);
    std::printf("  driver ON ...");
    std::fflush(stdout);
    const std::int64_t endHold = rt::monoNs() + static_cast<std::int64_t>(st.holdSeconds * 1e9);
    while (rt::monoNs() < endHold && !flags.stop) sleepMs(50);
    m.enable(false);
    std::printf(" OFF\n");
    char a = askYN("Did the cart resist while the driver was on?");
    if (a == 'q') return false;
    r.add("E1", "holding torque", a == 'y' ? Level::Pass : a == 's' ? Level::Skip : Level::Fail,
          a == 'y' ? "the motor held with ~ENABLE low"
                   : "no holding torque: no VMOT (F2, J2), motor wiring on J9, Vref ~0, the A4988 not seated, or ~ENABLE "
                     "(GPIO24 -> A4988 pin 9) not arriving");
    if (a == 'n') return true;
    a = askYN("And now, driver off: does it move freely?");
    if (a == 'q') return false;
    r.add("E1", "release", a == 'y' ? Level::Pass : a == 's' ? Level::Skip : Level::Fail,
          a == 'y' ? "free with ~ENABLE high" : "still held with ~ENABLE high: GPIO24 does not reach the A4988's pin 9");
    if (a == 'n') return true;

    // E2 direction
    const long nDir = static_cast<long>(st.dirTest * st.upm);
    if (!waitEnter("E2  The cart moves " + std::to_string(static_cast<int>(st.dirTest * 1e3)) +
                   " mm toward the LEFT switch at 10 mm/s, waits 1 s and comes back. Watch which way it goes first."))
        return false;
    auto anySwitch = [&] {
        if (m.pressed(kSwR) || m.pressed(kSwL)) { why = "a limit switch closed"; return true; }
        return false;
    };
    m.mode(1);
    m.enable(true);
    m.dir(1);
    why.clear();
    long done = m.pulses(nDir, slow, anySwitch, nullptr, &why);
    if (done == nDir && !flags.stop) {
        sleepMs(1000);
        m.dir(0);
        m.pulses(done, slow, anySwitch, nullptr, &why);
    }
    m.enable(false);
    if (done != nDir) {
        r.add("E2", "direction move", Level::Fail, "stopped after " + std::to_string(done) + " of " + std::to_string(nDir) + ": " + why);
        return true;
    }
    a = askYN("Did it go toward the LEFT switch first?");
    if (a == 'q') return false;
    if (a != 'y') {
        r.add("E2", "direction", a == 's' ? Level::Skip : Level::Fail,
              "DIR = 1 moved the cart toward the RIGHT: one motor coil is the other way round on J9 (swap J9.1 and J9.2). "
              "Homing and the soft limits rely on +x = LEFT. Stopping before the switch tests");
        return true;
    }
    r.add("E2", "direction", Level::Pass, "DIR = 1 = toward the LEFT switch (+x), as calib.x_sign = 1 expects");
    if (auto mm = askNumber("How far did it move, in mm (by eye or ruler)"))
        r.add("E2", "distance", core::judgeRange("moved", *mm, st.dirTest * 1e3 * 0.7, st.dirTest * 1e3 * 1.3, "mm"));

    // E3 step modes against the right switch
    if (!waitEnter("E3  The cart drives at 10 mm/s to the RIGHT switch (up to " + std::to_string(static_cast<int>(st.seekMax * 1e3)) +
                   " mm), backs off, then makes five 15 mm trips away and back, one per step mode (1/8, 1/4, 1/2, full, 1/8). "
                   "About a minute."))
        return false;
    const long seekMax = static_cast<long>(st.seekMax * st.upm), back = static_cast<long>(st.backoff * st.upm);
    m.mode(1);
    m.enable(true);
    m.dir(0);
    why.clear();
    if (m.seek(kSwR, kSwL, seekMax, slow, &why) < 0) {
        m.enable(false);
        r.add("E3", "right switch", Level::Fail, why);
        return true;
    }
    m.dir(1);
    if (!m.backOff(kSwR, back, slow, &why)) {
        m.enable(false);
        r.add("E3", "back-off", Level::Fail, why);
        return true;
    }
    std::vector<core::StepTrial> trials = {{"1/8", 1, 0, -1}, {"1/4", 2, 0, -1}, {"1/2", 4, 0, -1}, {"full", 8, 0, -1},
                                           {"1/8 again", 1, 0, -1}};
    std::size_t ran = 0;
    for (core::StepTrial& t : trials) {
        t.pulses = st.trialUsteps / t.usteps_per_pulse;
        m.mode(t.usteps_per_pulse);
        m.dir(1);
        why.clear();
        const long out = m.pulses(t.pulses, slow / t.usteps_per_pulse, [&] {
            if (m.pressed(kSwL)) { why = "the LEFT switch closed"; return true; }
            return false;
        }, nullptr, &why);
        m.mode(1);
        m.dir(0);
        if (out != t.pulses) break;
        why.clear();
        t.returned = m.seek(kSwR, kSwL, 8 * st.trialUsteps + 4 * back, slow, &why);
        std::printf("  %-9s %3ld pulses out, %4ld eighth-steps back%s\n", t.name, t.pulses, t.returned,
                    t.returned < 0 ? (" (" + why + ")").c_str() : "");
        ++ran;
        if (t.returned < 0) break;
        m.dir(1);
        if (!m.backOff(kSwR, back, slow, &why)) break;
    }
    if (ran < trials.size() || flags.stop) {
        m.enable(false);
        r.add("E3", "step modes", Level::Fail, "stopped after " + std::to_string(ran) + " trials: " + why);
        trials.resize(ran);
        for (const Judgement& j : core::judgeStepModes(trials, 0.10)) r.add("E3", "step mode", j);
        return true;
    }
    for (const Judgement& j : core::judgeStepModes(trials, 0.10)) r.add("E3", "step mode", j);

    // E4 span and back to the middle
    m.enable(false);
    if (!waitEnter("E4  The cart drives at 20 mm/s from the RIGHT switch to the LEFT switch (up to " +
                   std::to_string(static_cast<int>(st.seekMax * 1e3)) + " mm), then back to the middle."))
        return false;
    m.enable(true);
    m.dir(0);
    why.clear();
    if (m.seek(kSwR, kSwL, back * 4 + 200, slow, &why) < 0) {
        m.enable(false);
        r.add("E4", "right switch", Level::Fail, why);
        return true;
    }
    m.dir(1);
    why.clear();
    const long span = m.seek(kSwL, -1, seekMax, fast, &why);   // starts on the right switch: no "other" check
    if (span < 0) {
        m.enable(false);
        r.add("E4", "span", Level::Fail, "LEFT switch not reached: " + why);
        return true;
    }
    m.dir(0);
    m.pulses(span / 2, fast, nullptr, nullptr, &why);
    m.enable(false);
    const double rel = static_cast<double>(span) / static_cast<double>(st.spanUsteps) - 1.0;
    char s[260];
    std::snprintf(s, sizeof s, "%ld eighth-steps switch to switch = %.3f m at %.0f usteps/m; config balance.span_usteps = %ld (%+.1f %%)",
                  span, span / st.upm, st.upm, st.spanUsteps, rel * 100);
    r.add("E4", "span", std::fabs(rel) <= 0.05 ? Level::Pass : Level::Warn,
          std::string(s) + (std::fabs(rel) <= 0.05 ? ""
                                                   : ". The step modes passed, so the travel itself changed (new carriage?): "
                                                     "measure the switch-to-switch distance and set balance.span_usteps"));
    r.add("E4", "end position", Level::Info, "cart back in the middle, driver off");
    return true;
}

}

int runPcbTest(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    Settings st;
    st.stage = cfg.getString("pcbtest.stage");
    st.logDir = cfg.getString("log.dir");
    st.loadSeconds = cfg.getDouble("pcbtest.load_seconds");
    st.vSlow = cfg.getDouble("pcbtest.v_slow");
    st.vSpan = cfg.getDouble("pcbtest.v_span");
    st.seekMax = cfg.getDouble("pcbtest.seek_max");
    st.dirTest = cfg.getDouble("pcbtest.dir_test");
    st.backoff = cfg.getDouble("pcbtest.backoff");
    st.holdSeconds = cfg.getDouble("pcbtest.hold_seconds");
    st.trialUsteps = cfg.getInt("pcbtest.trial_usteps");
    st.requireD = cfg.getBool("pcbtest.require_stage_d");
    st.upm = cfg.getDouble("axis.usteps_per_m");
    st.cpr = cfg.getInt("encoder.counts_per_rev");
    st.spanUsteps = cfg.getInt("balance.span_usteps");
    const std::string chipName = cfg.getString("pcbtest.chip");
    const bool allowReal = cfg.getBool("pcbtest.allow_real_chip");
    if (st.stage != "C" && st.stage != "D" && st.stage != "E") {
        std::cerr << "pcb-test: pcbtest.stage must be C, D or E\n";
        return 2;
    }
    // Hard limits, whatever the config says: the test moves slowly and not far.
    if (!(st.vSlow > 0 && st.vSlow <= 0.02 && st.vSpan > 0 && st.vSpan <= 0.03 && st.seekMax > 0 && st.seekMax <= 0.6 &&
          st.dirTest > 0 && st.dirTest <= 0.03 && st.backoff > 0 && st.backoff <= 0.01 && st.trialUsteps > 0 &&
          st.trialUsteps <= 400 && st.holdSeconds > 0 && st.holdSeconds <= 10 && st.loadSeconds >= 0 && st.loadSeconds <= 120)) {
        std::cerr << "pcb-test: pcbtest.* out of the test's safe range (v_slow <= 20 mm/s, v_span <= 30 mm/s, seek_max <= 0.6 m, "
                     "dir_test <= 30 mm, backoff <= 10 mm, trial_usteps <= 400, hold <= 10 s)\n";
        return 2;
    }
    const std::pair<const char*, int> pins[] = {{"pins.step", kStep}, {"pins.dir", kDir}, {"pins.en", kEn}, {"pins.ms1", kMs1},
                                                {"pins.ms2", kMs2}, {"pins.limit_right", kSwR}, {"pins.limit_left", kSwL},
                                                {"pins.enc_a", kEncA}, {"pins.enc_b", kEncB}};
    for (const auto& p : pins)
        if (cfg.getInt(p.first) != p.second) {
            std::cerr << "pcb-test: " << p.first << " = " << cfg.getInt(p.first) << " but the PCB (pcb/PCB.kicad_pcb) wires GPIO"
                      << p.second << ". Refusing: fix the config or the board first.\n";
            return 2;
        }

    gpiod_chip* chip = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip) { std::cerr << "pcb-test: cannot open " << chipName << "\n"; return 1; }
    const std::string label = gpiod_chip_label(chip);
    const bool mock = label == "gpio-mockup-A";
    if (!mock && !allowReal) {
        std::cerr << "pcb-test: refusing " << chipName << " ('" << label << "'): the real board needs pcbtest.allow_real_chip=true "
                  << "(./pcbtest.sh after its checklist)\n";
        gpiod_chip_close(chip);
        return 2;
    }
    std::printf("pcb-test stage %s on %s (%s). Answers are typed; q or Ctrl-C stops (driver off). Report: logs/<time>_pcb-test-%s.\n",
                st.stage.c_str(), chipName.c_str(), label.c_str(), st.stage.c_str());

    const std::string dir = runlog::makeRunDir(st.logDir, "pcb-test-" + st.stage);
    Report r;
    bool completed = false;
    {
        Gpio g(chip);
        try {
            if (st.stage == "C") completed = stageC(g, mock, r, st);
            else if (st.stage == "D") completed = stageD(chip, g, r, st, dir);
            else completed = stageE(g, r, st);
        } catch (const std::exception& e) {
            r.add(st.stage, "error", Level::Fail, e.what());
        }
        restoreSafe(g);   // driver off, MS = 1/8, STEP/DIR low, the rest inputs with their power-on pulls
    }
    activeEnLine = nullptr;
    gpiod_chip_close(chip);
    if (flags.stop) completed = false;

    const std::string result = r.result(completed);
    r.summary();
    r.write(dir + "/report.txt", "pcb-test stage " + st.stage + ": " + result);
    runlog::writeMeta(dir, "pcb-test", cmdLine, cfg, configPath,
                      {{"stage", st.stage},
                       {"chip", chipName + " (" + label + ")"},
                       {"result", result},
                       {"pass", std::to_string(r.count(Level::Pass))},
                       {"warn", std::to_string(r.count(Level::Warn))},
                       {"fail", std::to_string(r.count(Level::Fail))},
                       {"skip", std::to_string(r.count(Level::Skip))}});
    std::printf("pcb-test stage %s: %s\n  -> %s\n", st.stage.c_str(), result.c_str(), dir.c_str());
    return r.count(Level::Fail) ? 3 : completed ? 0 : 4;
}

}
