#pragma once

#include <stdint.h>

uint64_t timeMillis();

#define EACH_N_TICKS(N, counter, code_blk) \
    if ((tick - (counter)) >= (N)) {        \
        (counter) = tick;                   \
        code_blk                            \
    }
