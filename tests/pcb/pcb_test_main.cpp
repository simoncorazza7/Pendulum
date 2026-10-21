// app::runPcbTest (the same code as `pendctl --mode pcb-test`) linked against the virtual board in
// tests/pcb/fake_gpiod.cpp instead of libgpiod. tests/pcb/run_pcb_fake.sh.
//   build/pcb_test_fake [--config FILE] [--set key=value ...]

#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "app/modes.h"
#include "common/config.h"

namespace app {
Flags flags;
std::atomic<Axis*> activeAxis{nullptr};
std::atomic<gpiod_line*> activeEnLine{nullptr};
}

int main(int argc, char** argv) {
    std::signal(SIGINT, [](int) { app::flags.stop = true; });
    std::string configPath = "config/pendctl.conf", cmdLine = "pcb_test_fake";
    std::vector<std::string> sets;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) configPath = argv[++i];
        else if (a == "--set" && i + 1 < argc) sets.push_back(argv[++i]);
        else { std::cerr << "usage: pcb_test_fake [--config FILE] [--set key=value ...]\n"; return 2; }
    }
    try {
        Config cfg = Config::fromFile(configPath);
        for (const auto& s : sets) {
            cfg.setFromAssignment(s);
            cmdLine += " --set " + s;
        }
        return app::runPcbTest(cfg, configPath, cmdLine);
    } catch (const std::exception& e) {
        std::cerr << "pcb_test_fake: " << e.what() << "\n";
        return 1;
    }
}
