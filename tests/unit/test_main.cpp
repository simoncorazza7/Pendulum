#include "check.h"

int main() {
    int failedTests = 0;
    for (const auto& t : check::registry()) {
        int before = check::failures();
        t.fn();
        bool ok = check::failures() == before;
        if (!ok) ++failedTests;
        std::cout << (ok ? "  pass " : "  FAIL ") << t.name << "\n";
    }
    std::cout << check::registry().size() - failedTests << "/" << check::registry().size()
              << " tests passed\n";
    return failedTests ? 1 : 0;
}
