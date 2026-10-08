# Build on the Pi:  make            (build/main, build/pendctl, build/unit_tests)
#                   make test       (unit tests only, no hardware)
# build/main uses exactly the flags of Kompilierer.sh (which still builds ./main).
# Running build/main or GPIO modes of build/pendctl is a hardware action (see CLAUDE.md).

CXX       ?= g++
BUILD     := build
GIT_HASH  := $(shell git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
GIT_DIRTY := $(shell git status --porcelain --untracked-files=no 2>/dev/null | grep -q . && echo 1 || echo 0)

APP_FLAGS := -std=c++17 -Wall -Wextra -O2 -g -Isrc -I. -DGIT_HASH=\"$(GIT_HASH)\" -DGIT_DIRTY=$(GIT_DIRTY)

MAIN_SRC  := main.cpp encoder.cpp axis.cpp pendel.cpp safety.cpp
MAIN_HDR  := axis.h encoder.h pendel.h safety.h SafeQueue.h
COMMON    := src/common/config.cpp src/common/runlog.cpp
CORE      := src/core/pid.cpp src/core/supervisor.cpp src/core/units.cpp src/core/swingfit.cpp
CORE_H    := src/core/pid.h src/core/supervisor.h src/core/units.h src/core/swingfit.h
HW        := src/hw/sim_hw.cpp
HW_H      := src/hw/hardware.h src/hw/sim_hw.h
COMMON_H  := src/common/config.h src/common/runlog.h src/rt/ringlog.h
PENDCTL   := src/app/pendctl.cpp src/app/selftest.cpp src/app/timing_probe.cpp src/app/encoder_log.cpp \
             src/rt/rt.cpp axis.cpp safety.cpp
PENDCTL_H := src/app/modes.h src/rt/rt.h src/rt/latency.h axis.h safety.h SafeQueue.h
TESTS     := tests/unit/test_main.cpp tests/unit/test_ringlog.cpp tests/unit/test_config.cpp tests/unit/test_pid.cpp \
             tests/unit/test_supervisor.cpp tests/unit/test_swingfit.cpp tests/unit/test_simhw.cpp

.PHONY: all test clean
all: $(BUILD)/main $(BUILD)/pendctl $(BUILD)/unit_tests

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/main: $(MAIN_SRC) $(MAIN_HDR) | $(BUILD)
	rm -f $@
	$(CXX) -Wall -std=c++17 -o $@ $(MAIN_SRC) -lgpiod -lgpiodcxx -pthread

$(BUILD)/pendctl: $(PENDCTL) $(PENDCTL_H) $(COMMON) $(COMMON_H) | $(BUILD)
	rm -f $@
	$(CXX) $(APP_FLAGS) -o $@ $(PENDCTL) $(COMMON) -lgpiod -pthread

$(BUILD)/unit_tests: $(TESTS) tests/unit/check.h $(COMMON) $(COMMON_H) $(CORE) $(CORE_H) $(HW) $(HW_H) tests/golden/*.csv | $(BUILD)
	rm -f $@
	$(CXX) $(APP_FLAGS) -Itests/unit -o $@ $(TESTS) $(COMMON) $(CORE) $(HW) -pthread

test: $(BUILD)/unit_tests
	$(BUILD)/unit_tests

clean:
	rm -f $(BUILD)/main $(BUILD)/pendctl $(BUILD)/unit_tests
