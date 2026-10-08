#ifndef SAFETY_H
#define SAFETY_H

#include <functional>

// Process-level safety for programs that drive the rig (Stage 0.3).
//
// Call order:
//   1. blockTerminationSignals()   first thing in main(), before any thread exists
//   2. construct the hardware objects (they start threads, which inherit the signal mask)
//   3. installFatalHandlers() and startTerminationWatcher()
namespace safety {

// Blocks SIGINT/SIGTERM in the calling thread and every thread created after it, so only the
// watcher thread receives them. Ignores SIGPIPE (dead gnuplot pipe, vanished TCP client).
void blockTerminationSignals();

// On SIGSEGV/SIGBUS/SIGILL/SIGFPE/SIGABRT: calls emergencyOff() from the signal handler, then
// re-raises the signal with its default action. emergencyOff must be async-signal-safe
// (here: one GPIO ioctl that sets EN = 1).
void installFatalHandlers(void (*emergencyOff)());

// Starts a thread that waits for SIGINT/SIGTERM and calls onTerminate(sig) in normal context.
// If onTerminate returns, the process ends with _Exit(128 + sig).
void startTerminationWatcher(std::function<void(int)> onTerminate);

}

#endif
