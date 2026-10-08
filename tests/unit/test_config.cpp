#include <string>

#include "check.h"
#include "common/config.h"

namespace {
const char* kText =
    "# comment line\n"
    "\n"
    "loop.ts = 0.002        # s\n"
    "log.capacity = 600000  # records\n"
    "log.dir = logs\n"
    "flag = true\n"
    "typo.key = 1\n";
}

TEST(config_parses_values_and_units) {
    Config c = Config::fromString(kText);
    CHECK_NEAR(c.getDouble("loop.ts"), 0.002, 1e-15);
    CHECK_EQ(c.getInt("log.capacity"), 600000L);
    CHECK_EQ(c.getString("log.dir"), std::string("logs"));
    CHECK(c.getBool("flag"));
    std::string dump = c.dumpUsed();
    CHECK(dump.find("loop.ts = 0.002    # s") != std::string::npos);
}

TEST(config_missing_and_malformed_throw) {
    Config c = Config::fromString(kText);
    CHECK_THROWS(c.getDouble("nope"));
    CHECK_THROWS(c.getInt("log.dir"));
    CHECK_THROWS(c.getInt("loop.ts"));          // "0.002" is not an integer
    CHECK_THROWS(Config::fromString("no equals sign\n"));
    CHECK_THROWS(Config::fromString("a = 1\na = 2\n"));
}

TEST(config_defaults_recorded) {
    Config c = Config::fromString(kText);
    CHECK_NEAR(c.getDouble("absent", 1.5), 1.5, 0);
    CHECK(c.dumpUsed().find("absent = 1.5    (default)") != std::string::npos);
}

TEST(config_overrides_and_unused_keys) {
    Config c = Config::fromString(kText);
    c.setFromAssignment("loop.ts=0.001");
    CHECK_NEAR(c.getDouble("loop.ts"), 0.001, 1e-15);
    CHECK(c.dumpUsed().find("(override)") != std::string::npos);
    c.getInt("log.capacity");
    c.getString("log.dir");
    c.getBool("flag");
    auto unused = c.unusedKeys();
    CHECK_EQ(unused.size(), 1u);
    CHECK_EQ(unused.at(0), std::string("typo.key"));
    CHECK_THROWS(c.setFromAssignment("noequals"));
}
