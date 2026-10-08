#include "safety.h"

#include <csignal>
#include <cstdlib>
#include <pthread.h>
#include <thread>
#include <unistd.h>

namespace safety {
namespace {

void (*g_emergencyOff)() = nullptr;

void fatalHandler(int sig) {
    if (g_emergencyOff) g_emergencyOff();
    const char msg[] = "\nFatal signal: driver disabled (EN = 1)\n";
    (void)!write(STDERR_FILENO, msg, sizeof msg - 1);
    raise(sig); // SA_RESETHAND restored the default action: crash as usual
}

sigset_t terminationSignals() {
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    return set;
}

}

void blockTerminationSignals() {
    sigset_t set = terminationSignals();
    pthread_sigmask(SIG_BLOCK, &set, nullptr);
    std::signal(SIGPIPE, SIG_IGN);
}

void installFatalHandlers(void (*emergencyOff)()) {
    g_emergencyOff = emergencyOff;
    struct sigaction sa {};
    sa.sa_handler = fatalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESETHAND | SA_NODEFER;
    for (int sig : {SIGSEGV, SIGBUS, SIGILL, SIGFPE, SIGABRT}) sigaction(sig, &sa, nullptr);
}

void startTerminationWatcher(std::function<void(int)> onTerminate) {
    std::thread([onTerminate] {
        sigset_t set = terminationSignals();
        int sig = SIGTERM;
        sigwait(&set, &sig);
        onTerminate(sig);
        std::_Exit(128 + sig);
    }).detach();
}

}
