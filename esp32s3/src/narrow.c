/* Tables for narrow.h. */
#include "narrow.h"

/* cos and sin of 2 pi m / 16, Q15. */
static const int16_t unit_cos[16] = {32767, 30274, 23170, 12540, 0, -12540, -23170, -30274,
                                     -32767, -30274, -23170, -12540, 0, 12540, 23170, 30274};
#define UNIT_SIN(m) unit_cos[((m) + 12u) & 15u]

/* Low-pass to 1.6 MHz at 16 Msps, Hamming window, Q14; symmetric. */
static const int16_t lowpass[12] = {-27, 62, 476, 1428, 2676, 3577, 3577, 2676, 1428, 476, 62, -27};

void narrow_reset(narrow_state *s, int32_t k)
{
    s->k = k;
    for (unsigned b = 0; b < 2; b++) {
        for (unsigned i = 0; i < 8; i++) {
            unsigned m = (unsigned)(k * (int32_t)(8u * b + i)) & 15u;
            s->cos[b][i] = unit_cos[m];
            s->sin[b][i] = UNIT_SIN(m);
        }
    }
    /* Lane L of the sixteen holds pair n - 8 + L. The output from the
     * twelve pairs ending at n + 3 weighs lane L by tap 11 - L; the one
     * ending at n + 7, by tap 15 - L. */
    for (unsigned l = 0; l < 16; l++) {
        s->taps[0][l] = l <= 11u ? lowpass[11u - l] : 0;
        s->taps[1][l] = l >= 4u ? lowpass[15u - l] : 0;
    }
    for (unsigned i = 0; i < 8; i++) {
        s->hist[0][i] = s->hist[1][i] = 0;
        s->qmask[i] = 0xFFC0u;
    }
    s->round = 1 << 18; /* half of the final shift by 19 */
}
