// pendctl: controller runtime for the Pendel rig.
//
// Modes:
//   log-selftest   fixed-rate loop logging a synthetic signal; no GPIO (Stage 0.4)
//   timing-probe   RT control loop + the Axis step thread on a GPIO chip, latency histograms
//                  (Stage 0.5). Refuses any chip but gpio-mockup unless probe.allow_real_chip=true.
//   encoder-log    records every raw encoder edge event (Stage 0.6); inputs only. Refuses the real
//                  chip unless enc.allow_real_chip=true.
//   move           point-to-point bring-up move (Stage 0.8/0.9); refuses the real chip unless
//                  move.allow_real_chip=true.
//   shadow         Stage 1 step 1: balance runtime on the real sensors, motor lines never requested;
//                  refuses the real chip unless shadow.allow_real_chip=true.
//   damp           Stage 1 step 2: hanging active-damping test (moves the cart); refuses the real
//                  chip unless damp.allow_real_chip=true.
//   balance        Stage 1 step 3: balance session (homes, balances, re-centres; interactive). Refuses
//                  the real chip unless balance.allow_real_chip=true (demo.sh).
// Usage:
//   pendctl --mode MODE [--config config/pendctl.conf] [--set key=value ...]
//
// SIGINT/SIGTERM: the mode stops its loop, brakes, sets EN = 1 and still writes its logs. If that
// has not happened within 1 s, the watcher sets EN = 1 itself. Fatal signals: EN = 1 at once.

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "app/modes.h"
#include "axis.h"
#include "safety.h"

namespace app {
Flags flags;
std::atomic<Axis*> activeAxis{nullptr};
}

namespace {

void emergencyOff() {
    if (Axis* a = app::activeAxis.load()) a->disableDriver();
}

void onTerminate(int sig) {
    std::cerr << "\npendctl: signal " << sig << ", stopping\n";
    app::flags.stop = true;
    for (int i = 0; i < 100 && !app::flags.motionSafe; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!app::flags.motionSafe) {
        std::cerr << "pendctl: no safe stop within 1 s, driver off (EN = 1)\n";
        emergencyOff();
    }
    for (int i = 0; i < 3000 && !app::flags.done; ++i)   // let the mode write its logs (<= 30 s)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void usage() {
    std::cout << "Usage: pendctl --mode log-selftest|timing-probe|encoder-log|move|shadow|damp|balance [--config FILE] "
                 "[--set key=value ...]\n";
}

}

int main(int argc, char** argv) {
    safety::blockTerminationSignals();   // before any thread exists

    std::string mode;
    std::string configPath = "config/pendctl.conf";
    std::vector<std::string> overrides;
    std::string cmdLine;
    for (int i = 0; i < argc; ++i) cmdLine += (i ? " " : "") + std::string(argv[i]);

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--mode" && i + 1 < argc) mode = argv[++i];
        else if (a == "--config" && i + 1 < argc) configPath = argv[++i];
        else if (a == "--set" && i + 1 < argc) overrides.push_back(argv[++i]);
        else { usage(); return a == "--help" ? 0 : 2; }
    }

    safety::installFatalHandlers(emergencyOff);
    safety::startTerminationWatcher(onTerminate);

    int rc = 2;
    try {
        Config cfg = Config::fromFile(configPath);
        for (const auto& o : overrides) cfg.setFromAssignment(o);
        if (mode == "log-selftest") rc = app::runSelftest(cfg, configPath, cmdLine);
        else if (mode == "timing-probe") rc = app::runTimingProbe(cfg, configPath, cmdLine);
        else if (mode == "encoder-log") rc = app::runEncoderLog(cfg, configPath, cmdLine);
        else if (mode == "move") rc = app::runMove(cfg, configPath, cmdLine);
        else if (mode == "shadow") rc = app::runShadow(cfg, configPath, cmdLine);
        else if (mode == "damp") rc = app::runDamp(cfg, configPath, cmdLine);
        else if (mode == "balance") rc = app::runBalance(cfg, configPath, cmdLine);
        else usage();
    } catch (const std::exception& e) {
        std::cerr << "pendctl: " << e.what() << "\n";
        emergencyOff();
        rc = 1;
    }
    app::flags.done = true;
    return rc;
}
