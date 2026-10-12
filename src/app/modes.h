#ifndef PENDEL_MODES_H
#define PENDEL_MODES_H

#include <atomic>
#include <string>

#include "common/config.h"

class Axis;

namespace app {

// Shared between the modes and the signal watcher in pendctl.cpp.
struct Flags {
    std::atomic<bool> stop{false};        // set on SIGINT/SIGTERM: modes end their loop
    std::atomic<bool> motionSafe{true};   // false while an Axis may be moving; true after halt + EN = 1
    std::atomic<bool> done{false};        // mode finished writing its logs
};
extern Flags flags;
extern std::atomic<Axis*> activeAxis;     // the fatal-signal handler sets its EN = 1

int runSelftest(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runTimingProbe(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runEncoderLog(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runMove(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runShadow(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runDamp(const Config& cfg, const std::string& configPath, const std::string& cmdLine);
int runBalance(const Config& cfg, const std::string& configPath, const std::string& cmdLine);

}

#endif
