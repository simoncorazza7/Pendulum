// The libgpiod v1 API subset pendctl's pcb-test and hw::EdgeReader use, for the virtual board in
// tests/pcb/fake_gpiod.cpp (tests/pcb/run_pcb_fake.sh). Same names and signatures as libgpiod 1.6.
#pragma once
#include <time.h>

struct gpiod_chip;
struct gpiod_line;
struct gpiod_line_event {
    struct timespec ts;
    int event_type;
};
enum { GPIOD_LINE_EVENT_RISING_EDGE = 1, GPIOD_LINE_EVENT_FALLING_EDGE };
enum {
    GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN = 1,
    GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE = 2,
    GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW = 4,
    GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE = 8,
    GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN = 16,
    GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP = 32
};

extern "C" {
struct gpiod_chip* gpiod_chip_open_by_name(const char* name);
void gpiod_chip_close(struct gpiod_chip* chip);
const char* gpiod_chip_label(struct gpiod_chip* chip);
struct gpiod_line* gpiod_chip_get_line(struct gpiod_chip* chip, unsigned int offset);
int gpiod_line_request_input_flags(struct gpiod_line* line, const char* consumer, int flags);
int gpiod_line_request_output(struct gpiod_line* line, const char* consumer, int default_val);
int gpiod_line_request_both_edges_events(struct gpiod_line* line, const char* consumer);
int gpiod_line_event_get_fd(struct gpiod_line* line);
int gpiod_line_event_read_multiple(struct gpiod_line* line, struct gpiod_line_event* events, unsigned int num_events);
int gpiod_line_set_value(struct gpiod_line* line, int value);
int gpiod_line_get_value(struct gpiod_line* line);
void gpiod_line_release(struct gpiod_line* line);
bool gpiod_line_is_used(struct gpiod_line* line);
}
