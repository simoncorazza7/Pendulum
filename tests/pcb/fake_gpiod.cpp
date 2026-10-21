// A virtual carrier PCB behind the libgpiod v1 API, so pendctl's pcb-test runs unmodified against it
// (tests/pcb/run_pcb_fake.sh). Never touches real hardware.
//
// Electrical model per GPIO node (lines joined by injected bridges form one node): the Pi's internal pull (50 kOhm,
// persists after release), the board's resistors (pcb/PCB.kicad_pcb: 10k / 4.7k pull-ups, the Pi's 1.8k on GPIO2/3,
// the A4988's 100k / 50k pull-downs on MS1 / MS2), driven outputs (ideal; they keep driving after release, as with
// persist_gpio_outputs), hard sources (shorts to GND / 3V3, a pressed switch, the encoder's open collector pulling
// low; these beat outputs). Level = node voltage > 1.65 V.
// A simulated cart moves one step per STEP rising edge while ~ENABLE reads low, the motor is plugged in and F2 is in:
// eighth-steps per pulse from MS1/MS2 (A4988 table), direction from DIR; it closes the limit switches at its ends.
// Environment:
//   FAKE_PLUGGED  switches,enc,motor,f2         what is connected (default: nothing)
//   FAKE_FAULTS   bridge:20-21, gnd:27, v33:4, v5:2, open:10, used:14, msswap, dirinv, swapsw, swapenc
//   FAKE_PRESS    right@2.0+1.0,left@5+1        scripted presses (s since start, duration)
//   FAKE_TURNS    19:-2400:3,25:2400:3          scripted pendulum turns (start s, counts, duration s)
//   FAKE_CART     start position, eighth-steps from the right switch (default span/2); FAKE_SPAN (default 6104)
// At exit it prints one FAKE-END line (final pin levels, cart, steps) to stderr for the test script.

#include "gpiod.h"

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

struct gpiod_line {
    int offset;
};
struct gpiod_chip {
    int dummy;
};

namespace {

constexpr int kN = 28, kStep = 27, kDir = 22, kEn = 24, kMs1 = 20, kMs2 = 21, kSwR = 2, kSwL = 9, kEncA = 10, kEncB = 19;

struct Line {
    gpiod_line handle{0};
    int req = 0;              // 0 free, 1 input, 2 output, 3 events
    int pull = 0;             // +1 up, -1 down, 0 none (persists)
    bool driving = false;     // drives driveVal (persists after release of an output)
    int driveVal = 0;
    int extUp = 0;            // ohms to 3V3 on the board (0 = none)
    int extDown = 0;          // ohms to GND
    int shortTo = -1;         // -1 none, 0 GND, 1 3V3 / 5V
    bool used = false;
    int lastLevel = 0;
    int pipe[2] = {-1, -1};
    std::deque<gpiod_line_event> q;
};

struct Press {
    bool right;
    double t0, dur;
};
struct Turn {
    double t0;
    long counts;
    double dur;
};

struct Board {
    std::mutex mu;
    Line l[kN];
    int parent[kN];
    gpiod_chip chip{0};
    std::set<std::string> plugged, flags;
    std::vector<Press> presses;
    std::vector<Turn> turns;
    long span = 6104;
    long cart = 3052;
    bool rightLatched = false, leftLatched = false;
    long steps = 0, stepsDisabled = 0, conflicts = 0, enc = 0;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::thread turner;
    std::atomic<bool> run{true};

    int find(int x) { return parent[x] == x ? x : parent[x] = find(parent[x]); }
    double now() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

    Board() {
        for (int i = 0; i < kN; ++i) {
            l[i].handle.offset = i;
            l[i].pull = i <= 8 ? +1 : -1;   // BCM2711 power-on pulls
            parent[i] = i;
        }
        l[2].extUp = 1525;                   // R4 10k || the Pi's 1.8k
        l[3].extUp = 1800;
        l[9].extUp = 10000;
        l[10].extUp = 4700;
        l[19].extUp = 4700;
        l[kMs1].extDown = 100000;            // inside the A4988
        l[kMs2].extDown = 50000;
        auto list = [](const char* env) {
            std::vector<std::string> v;
            const char* s = std::getenv(env);
            if (!s) return v;
            std::string x = s, cur;
            for (char c : x) {
                if (c == ',') { if (!cur.empty()) v.push_back(cur); cur.clear(); }
                else if (c != ' ') cur += c;
            }
            if (!cur.empty()) v.push_back(cur);
            return v;
        };
        for (const auto& p : list("FAKE_PLUGGED")) plugged.insert(p);
        for (const auto& f : list("FAKE_FAULTS")) {
            int a = -1, b = -1;
            if (std::sscanf(f.c_str(), "bridge:%d-%d", &a, &b) == 2) parent[find(a)] = find(b);
            else if (std::sscanf(f.c_str(), "gnd:%d", &a) == 1) l[a].shortTo = 0;
            else if (std::sscanf(f.c_str(), "v33:%d", &a) == 1 || std::sscanf(f.c_str(), "v5:%d", &a) == 1) l[a].shortTo = 1;
            else if (std::sscanf(f.c_str(), "open:%d", &a) == 1) l[a].extUp = l[a].extDown = 0;
            else if (std::sscanf(f.c_str(), "used:%d", &a) == 1) l[a].used = true;
            else flags.insert(f);
        }
        for (const auto& p : list("FAKE_PRESS")) {
            char which[16] = {};
            double t = 0, d = 0;
            if (std::sscanf(p.c_str(), "%15[a-z]@%lf+%lf", which, &t, &d) == 3) presses.push_back({std::string(which) == "right", t, d});
        }
        for (const auto& p : list("FAKE_TURNS")) {
            double t = 0, d = 0;
            long n = 0;
            if (std::sscanf(p.c_str(), "%lf:%ld:%lf", &t, &n, &d) == 3) turns.push_back({t, n, d});
        }
        if (const char* s = std::getenv("FAKE_SPAN")) span = std::atol(s);
        cart = span / 2;
        if (const char* s = std::getenv("FAKE_CART")) cart = std::atol(s);
        if (!turns.empty()) turner = std::thread([this] { turnLoop(); });
    }

    ~Board() {
        run = false;
        if (turner.joinable()) turner.join();
        std::lock_guard<std::mutex> g(mu);
        std::fprintf(stderr, "FAKE-END EN=%d EN_driving=%d MS1=%d MS2=%d STEP=%d DIR=%d cart=%ld steps=%ld steps_disabled=%ld "
                             "conflicts=%ld encoder=%ld\n",
                     level(kEn), l[kEn].driving ? 1 : 0, level(kMs1), level(kMs2), level(kStep), level(kDir), cart, steps,
                     stepsDisabled, conflicts, enc);
    }

    bool pressedPhysical(bool right) {
        if (!plugged.count("switches")) return false;
        const double t = now();
        for (const Press& p : presses)
            if (p.right == right && t >= p.t0 && t < p.t0 + p.dur) return true;
        return right ? rightLatched : leftLatched;
    }
    void updateCartSwitches() {
        if (cart <= 0) rightLatched = true;
        else if (cart > 15) rightLatched = false;
        if (cart >= span) leftLatched = true;
        else if (cart < span - 15) leftLatched = false;
    }
    // Hard source on line i: -1 none, 0 GND, 1 high.
    int hard(int i) {
        if (l[i].shortTo >= 0) return l[i].shortTo;
        const bool sw = flags.count("swapsw") != 0;
        if (i == kSwR && pressedPhysical(!sw)) return 0;
        if (i == kSwL && pressedPhysical(sw)) return 0;
        if (plugged.count("enc") && (i == kEncA || i == kEncB)) {
            const int q = static_cast<int>(((enc % 4) + 4) % 4);
            const int A = (q == 2 || q == 3), B = (q == 1 || q == 2);   // hw::EdgeReader: B leading A = +1
            const bool ab = flags.count("swapenc") != 0;
            const int bit = (i == kEncA) != ab ? A : B;
            if (bit == 0) return 0;   // open collector: pulls low, or lets go
        }
        return -1;
    }
    int level(int i) {
        const int root = find(i);
        int hi = 0, lo = 0, outHi = 0, outLo = 0;
        double g = 0, gv = 0;
        for (int k = 0; k < kN; ++k) {
            if (find(k) != root) continue;
            const int h = hard(k);
            if (h == 0) ++lo;
            if (h == 1) ++hi;
            if (l[k].driving) (l[k].driveVal ? outHi : outLo)++;
            if (l[k].req != 2 && !l[k].driving && l[k].pull) { g += 1.0 / 50000; gv += (l[k].pull > 0 ? 3.3 : 0.0) / 50000; }
            if (l[k].extUp) { g += 1.0 / l[k].extUp; gv += 3.3 / l[k].extUp; }
            if (l[k].extDown) g += 1.0 / l[k].extDown;
        }
        int v;
        if (lo || hi) {
            v = lo ? 0 : 1;
            if (outHi && lo) ++conflicts;
        } else if (outHi || outLo) {
            if (outHi && outLo) ++conflicts;
            v = outLo ? 0 : 1;
        } else if (g > 0) {
            v = gv / g > 1.65 + 1e-9 ? 1 : 0;
        } else {
            v = l[i].lastLevel;
        }
        l[i].lastLevel = v;
        return v;
    }
    void stepPulse() {
        if (!plugged.count("motor") || !plugged.count("f2")) return;
        if (level(kEn) != 0) { ++stepsDisabled; return; }
        int m1 = level(kMs1), m2 = level(kMs2);
        if (flags.count("msswap")) std::swap(m1, m2);
        const int upp = m1 && m2 ? 1 : !m1 && m2 ? 2 : m1 && !m2 ? 4 : 8;
        int d = level(kDir);
        if (flags.count("dirinv")) d = !d;
        cart += d ? upp : -upp;
        cart = std::max(-150L, std::min(span + 150, cart));
        ++steps;
        updateCartSwitches();
    }
    void pushEvent(int i, int newLevel) {
        Line& x = l[i];
        if (x.req != 3) return;
        gpiod_line_event e;
        clock_gettime(CLOCK_MONOTONIC, &e.ts);
        e.event_type = newLevel ? GPIOD_LINE_EVENT_RISING_EDGE : GPIOD_LINE_EVENT_FALLING_EDGE;
        x.q.push_back(e);
        const char c = 1;
        if (write(x.pipe[1], &c, 1) < 0) {}
    }
    void turnLoop() {
        std::size_t next = 0;
        while (run && next < turns.size()) {
            const Turn t = turns[next];
            while (run && now() < t.t0) std::this_thread::sleep_for(std::chrono::milliseconds(5));
            const long n = std::labs(t.counts);
            const int s = t.counts > 0 ? 1 : -1;
            for (long k = 0; k < n && run; ++k) {
                {
                    std::lock_guard<std::mutex> g(mu);
                    const int a0 = level(kEncA), b0 = level(kEncB);
                    enc += s;
                    const int a1 = level(kEncA), b1 = level(kEncB);
                    if (a1 != a0) pushEvent(kEncA, a1);
                    if (b1 != b0) pushEvent(kEncB, b1);
                }
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(t.dur * 1e6 / static_cast<double>(n))));
            }
            ++next;
        }
    }
};

Board& B() {
    static Board b;
    return b;
}

}

extern "C" {

gpiod_chip* gpiod_chip_open_by_name(const char*) { return &B().chip; }
void gpiod_chip_close(gpiod_chip*) {}
const char* gpiod_chip_label(gpiod_chip*) { return "gpio-mockup-A"; }

gpiod_line* gpiod_chip_get_line(gpiod_chip*, unsigned int offset) {
    return offset < static_cast<unsigned>(kN) ? &B().l[offset].handle : nullptr;
}

int gpiod_line_request_input_flags(gpiod_line* line, const char*, int flags) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    Line& x = b.l[line->offset];
    if (x.req || x.used) return -1;
    x.req = 1;
    x.driving = false;
    if (flags & GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) x.pull = +1;
    else if (flags & GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN) x.pull = -1;
    else if (flags & GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE) x.pull = 0;
    return 0;
}

int gpiod_line_request_output(gpiod_line* line, const char*, int v) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    Line& x = b.l[line->offset];
    if (x.req || x.used) return -1;
    const int prev = x.driving ? x.driveVal : 0;
    x.req = 2;
    x.driving = true;
    x.driveVal = v ? 1 : 0;
    if (line->offset == kStep && prev == 0 && x.driveVal == 1) b.stepPulse();
    return 0;
}

int gpiod_line_request_both_edges_events(gpiod_line* line, const char*) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    Line& x = b.l[line->offset];
    if (x.req || x.used) return -1;
    x.req = 3;
    x.driving = false;
    if (x.pipe[0] < 0) {
        if (pipe(x.pipe) != 0) return -1;
        fcntl(x.pipe[0], F_SETFL, O_NONBLOCK);
    }
    return 0;
}

int gpiod_line_event_get_fd(gpiod_line* line) { return B().l[line->offset].pipe[0]; }

int gpiod_line_event_read_multiple(gpiod_line* line, gpiod_line_event* ev, unsigned int n) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    Line& x = b.l[line->offset];
    unsigned k = 0;
    while (k < n && !x.q.empty()) {
        ev[k++] = x.q.front();
        x.q.pop_front();
        char c;
        if (read(x.pipe[0], &c, 1) < 0) {}
    }
    return static_cast<int>(k);
}

int gpiod_line_set_value(gpiod_line* line, int v) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    Line& x = b.l[line->offset];
    if (x.req != 2) return -1;
    const int prev = x.driveVal;
    x.driveVal = v ? 1 : 0;
    if (line->offset == kStep && prev == 0 && x.driveVal == 1) b.stepPulse();
    return 0;
}

int gpiod_line_get_value(gpiod_line* line) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    if (!b.l[line->offset].req) return -1;
    b.updateCartSwitches();
    return b.level(line->offset);
}

void gpiod_line_release(gpiod_line* line) {
    Board& b = B();
    std::lock_guard<std::mutex> g(b.mu);
    b.l[line->offset].req = 0;   // an output keeps driving (persist_gpio_outputs)
}

bool gpiod_line_is_used(gpiod_line* line) { return B().l[line->offset].used; }

}
