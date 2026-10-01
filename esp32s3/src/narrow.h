/*
 * One channel cut from the 16 MHz window at 4 Msps, with the ESP32-S3's
 * vector extension.
 *
 * Pairs are mixed down by k MHz (whole MHz, so the mixer repeats every 16
 * pairs), low-pass filtered to +-1.6 MHz by a 12-tap filter and kept at every
 * fourth pair. Eight pairs n..n+7 go in per step, from an address aligned to
 * 16 bytes, and two outputs come out, from the twelve pairs ending at n + 3
 * and at n + 7: output j of a channel started at pair s is centred on
 * s + 4j - 2.5. The filter's history lives in memory between steps, so
 * nothing else needs the vector registers kept.
 */
#pragma once

#include <stdint.h>

typedef struct {
    int16_t cos[2][8];   /* mixer for a group at pair n, by (n / 8) % 2, Q15 */
    int16_t sin[2][8];
    int16_t taps[2][16]; /* filter for each output, over the previous group and this one, Q14 */
    int16_t hist[2][8];  /* previous group's mixed I and Q, scaled by 32 */
    uint16_t qmask[8];
    int64_t round;       /* starting value of each output's accumulator */
    int32_t k;
} __attribute__((aligned(32))) narrow_state;

/* narrow.S depends on these. */
_Static_assert(__builtin_offsetof(narrow_state, sin) == 32, "narrow_state layout");
_Static_assert(__builtin_offsetof(narrow_state, taps) == 64, "narrow_state layout");
_Static_assert(__builtin_offsetof(narrow_state, hist) == 128, "narrow_state layout");
_Static_assert(__builtin_offsetof(narrow_state, qmask) == 160, "narrow_state layout");
_Static_assert(__builtin_offsetof(narrow_state, round) == 176, "narrow_state layout");

/* Starts a channel k MHz from the centre (-8..7), with an empty history. */
void narrow_reset(narrow_state *s, int32_t k);

/* Feeds `steps` groups of eight pairs from w (pair numbers n.., n a multiple
 * of 8, w aligned to 16 bytes) and writes two outputs per group, as 20-bit
 * pairs (I and Q each 10 bits, clamped to -512..511). */
void narrow_run(narrow_state *s, const volatile uint32_t *w, uint32_t n, uint32_t steps, uint32_t *out);
