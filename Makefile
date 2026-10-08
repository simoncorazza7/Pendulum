# Build on the Pi:  make            (build/main, build/pendctl, build/unit_tests)
#                   make test       (unit tests only, no hardware)
# build/main uses exactly the flags of Kompilierer.sh (which still builds ./main).
# Running build/main or GPIO modes of build/pendctl is a hardware action (see CLAUDE.md).

CXX       ?= g++
BUILD     := build
GIT_HASH  := $(shell git rev-parse --short=12 HEAD 2>/dev/null || echo unknown)
GIT_DIRTY := $(shell git status --porcelain --untracked-files=no 2>/dev/null | grep -q . && echo 1 || echo 0)

APP_FLAGS := -std=c++17 -Wall -Wextra -O2 -g -Isrc -DGIT_HASH=\"$(GIT_HASH)\" -DGIT_DIRTY=$(GIT_DIRTY)

MAIN_SRC  := main.cpp encoder.cpp axis.cpp pendel.cpp safety.cpp
MAIN_HDR  := axis.h encoder.h pendel.h safety.h SafeQueue.h
COMMON    := src/common/config.cpp src/common/runlog.cpp
COMMON_H  := src/common/config.h src/common/runlog.h src/rt/ringlog.h
TESTS     := tests/unit/test_main.cpp tests/unit/test_ringlog.cpp tests/unit/test_config.cpp

.PHONY: all test clean
all: $(BUILD)/main $(BUILD)/pendctl $(BUILD)/unit_tests

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/main: $(MAIN_SRC) $(MAIN_HDR) | $(BUILD)
	rm -f $@
	$(CXX) -Wall -std=c++17 -o $@ $(MAIN_SRC) -lgpiod -lgpiodcxx -pthread

$(BUILD)/pendctl: src/app/pendctl.cpp $(COMMON) $(COMMON_H) | $(BUILD)
	rm -f $@
	$(CXX) $(APP_FLAGS) -o $@ src/app/pendctl.cpp $(COMMON) -pthread

$(BUILD)/unit_tests: $(TESTS) tests/unit/check.h $(COMMON) $(COMMON_H) | $(BUILD)
	rm -f $@
	$(CXX) $(APP_FLAGS) -Itests/unit -o $@ $(TESTS) $(COMMON) -pthread

test: $(BUILD)/unit_tests
	$(BUILD)/unit_tests

clean:
	rm -f $(BUILD)/main $(BUILD)/pendctl $(BUILD)/unit_tests
