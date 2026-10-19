// Standalone remote-sim for machines without libgpiod (WSL, a Linux PC): the same app::runRemoteSim as
// `pendctl --mode remote-sim`, built from the portable sources plus the UDP link. tests/remote/run_remote_e2e.sh.
//   build/remote_sim_test [--config FILE] [--set key=value ...]

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
}

int main(int argc, char** argv) {
    std::signal(SIGINT, [](int) { app::flags.stop = true; });
    std::signal(SIGTERM, [](int) { app::flags.stop = true; });
    std::string configPath = "config/pendctl.conf", cmdLine = "remote_sim_test";
    std::vector<std::string> sets;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) configPath = argv[++i];
        else if (a == "--set" && i + 1 < argc) sets.push_back(argv[++i]);
        else { std::cerr << "usage: remote_sim_test [--config FILE] [--set key=value ...]\n"; return 2; }
    }
    try {
        Config cfg = Config::fromFile(configPath);
        for (const auto& s : sets) {
            cfg.setFromAssignment(s);
            cmdLine += " --set " + s;
        }
        return app::runRemoteSim(cfg, configPath, cmdLine);
    } catch (const std::exception& e) {
        std::cerr << "remote_sim_test: " << e.what() << "\n";
        return 1;
    }
}
