#ifndef PENDEL_CHECK_H
#define PENDEL_CHECK_H

// Minimal in-repo test harness (no external dependencies).
//   TEST(name) { CHECK(cond); CHECK_EQ(a, b); CHECK_NEAR(a, b, tol); CHECK_THROWS(expr); }
// All tests are linked into one binary; tests/unit/test_main.cpp runs them.

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace check {

struct Test { const char* name; std::function<void()> fn; };
inline std::vector<Test>& registry() { static std::vector<Test> r; return r; }
inline int& failures() { static int f = 0; return f; }
struct Register { Register(const char* n, std::function<void()> f) { registry().push_back({n, f}); } };

inline void fail(const char* file, int line, const std::string& msg) {
    std::cout << "    FAIL " << file << ":" << line << ": " << msg << "\n";
    ++failures();
}

}

#define TEST(name)                                                    \
    static void name();                                               \
    static check::Register name##_reg(#name, name);                   \
    static void name()

#define CHECK(cond) \
    do { if (!(cond)) check::fail(__FILE__, __LINE__, #cond); } while (0)

#define CHECK_EQ(a, b)                                                                  \
    do {                                                                                \
        auto va_ = (a); auto vb_ = (b);                                                 \
        if (!(va_ == vb_)) check::fail(__FILE__, __LINE__, std::string(#a " == " #b)); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                           \
    do {                                                                                \
        double va_ = (a), vb_ = (b);                                                    \
        if (!(std::fabs(va_ - vb_) <= (tol)))                                           \
            check::fail(__FILE__, __LINE__, std::string(#a " ~ " #b ": ") +             \
                        std::to_string(va_) + " vs " + std::to_string(vb_));            \
    } while (0)

#define CHECK_THROWS(expr)                                                              \
    do {                                                                                \
        bool threw_ = false;                                                            \
        try { (void)(expr); } catch (...) { threw_ = true; }                            \
        if (!threw_) check::fail(__FILE__, __LINE__, "expected exception: " #expr);     \
    } while (0)

#endif
