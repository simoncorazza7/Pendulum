#!/bin/bash
# Build and run the portable unit tests on the Windows PC (clang++ with the MSVC STL), from the repo
# root. The Pi uses `make test`. runlog/rt/GPIO code is POSIX-only and not part of this build.
set -e
cd "$(dirname "$0")/../.."
mkdir -p build
rm -f build/unit_tests_pc.exe
clang++ -std=c++17 -Wall -Wextra -O2 -Isrc -Itests/unit -o build/unit_tests_pc.exe \
    tests/unit/test_main.cpp tests/unit/test_ringlog.cpp tests/unit/test_config.cpp tests/unit/test_pid.cpp \
    tests/unit/test_supervisor.cpp tests/unit/test_swingfit.cpp \
    src/common/config.cpp src/core/pid.cpp src/core/supervisor.cpp src/core/units.cpp src/core/swingfit.cpp
build/unit_tests_pc.exe
