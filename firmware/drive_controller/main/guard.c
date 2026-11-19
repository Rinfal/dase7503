#include "guard.h"
#include <string.h>

void guard_stop(drive_guard_t *g) {
    g->armed = false;
    memset(g->duty, 0, sizeof(g->duty));
}

void guard_tick(drive_guard_t *g, int64_t now) {
    if (g->armed && (now < g->command_us || now - g->command_us >= 250000)) {
        guard_stop(g);
        g->timed_out = true;
        g->neutral_us = 0;
    }
}

bool guard_arm(drive_guard_t *g, int64_t now) {
    guard_tick(g, now);
    if (g->armed || !g->neutral_us || now < g->neutral_us || now - g->neutral_us > 200000) return false;
    memset(g->duty, 0, sizeof(g->duty));
    g->command_us = now;
    g->timed_out = false;
    g->armed = true;
    g->neutral_us = 0;
    return true;
}

bool guard_command(drive_guard_t *g, const int *duty, int64_t now) {
    guard_tick(g, now);
    bool neutral = true;
    for (int i = 0; i < 4; i++) {
        if (duty[i] < -600 || duty[i] > 600) {
            guard_stop(g);
            g->neutral_us = 0;
            return false;
        }
        neutral = neutral && duty[i] == 0;
    }
    if (neutral) g->neutral_us = now;
    if (!g->armed) return neutral;
    memcpy(g->duty, duty, sizeof(g->duty));
    g->command_us = now;
    return true;
}
