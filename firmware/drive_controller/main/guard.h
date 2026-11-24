#pragma once
#include <stdbool.h>
#include <stdint.h>

#define DRIVE_MAX_DUTY 800

typedef struct {
    bool armed;
    bool timed_out;
    int duty[4];
    int64_t command_us;
    int64_t neutral_us;
} drive_guard_t;

void guard_stop(drive_guard_t *g);
bool guard_command(drive_guard_t *g, const int *duty, int64_t now);
bool guard_arm(drive_guard_t *g, int64_t now);
void guard_tick(drive_guard_t *g, int64_t now);
