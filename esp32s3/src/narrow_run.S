/*
 * void narrow_run(narrow_state *s, const uint32_t *w, uint32_t n,
 *                 uint32_t steps, uint32_t *out)
 *
 * Feeds `steps` groups of eight pairs from w (pair numbers n.., n a multiple
 * of 8, w aligned to 16 bytes) through the channel filter and writes two
 * 20-bit outputs per group. See narrow.h for the state's layout.
 */
    .text
    .align 4
    .global narrow_run
    .type narrow_run, @function
narrow_run:
    entry a1, 32
    beqz a5, 2f
    extui a7, a4, 3, 1
    slli a7, a7, 4
    add a7, a2, a7          /* mixer for this group: cos at a7, sin 32 bytes on */
    addi a8, a2, 160
    ee.vld.128.ip q7, a8, 0 /* mask for Q (bits 6-15 of each half) */
    addi a8, a2, 64         /* taps */
    addi a9, a2, 128        /* history */
    addi a10, a2, 176       /* rounding for the final shift */
    movi a11, 6
    movi a12, 12
    movi a13, 16
    movi a14, 19
    loopnez a5, 1f
    /* Unpack: shifted left by 6, the low half of each pair is I * 64;
     * shifted by 12, the high half is Q * 64 above six bits of I. */
    ee.vld.128.ip q0, a3, 16
    ee.vld.128.ip q1, a3, 16
    wsr.sar a11
    ee.vsl.32 q2, q0
    ee.vsl.32 q3, q1
    wsr.sar a12
    ee.vsl.32 q4, q0
    ee.vsl.32 q5, q1
    ee.vunzip.16 q2, q3     /* q2 = I * 64 */
    ee.vunzip.16 q4, q5     /* q5 = Q * 64 + low bits of I */
    ee.vld.128.ip q0, a7, 32
    ee.vld.128.ip q1, a7, -32
    ee.andq q5, q5, q7      /* q5 = Q * 64 */
    /* Mix: (I + jQ)(c - js), each product scaled by 2^-16, so 32x. */
    wsr.sar a13
    ee.vmul.s16 q3, q2, q0
    ee.vmul.s16 q4, q5, q1
    ee.vmul.s16 q6, q5, q0
    ee.vmul.s16 q0, q2, q1
    xor a7, a7, a13         /* the next group's mixer */
    ee.vadds.s16 q3, q3, q4 /* q3 = mixed I */
    ee.vsubs.s16 q4, q6, q0 /* q4 = mixed Q */
    /* The previous group's mixed pairs out of the history, these in. */
    ee.vld.128.ip q0, a9, 16
    ee.vld.128.ip q1, a9, -16
    ee.vst.128.ip q3, a9, 16
    ee.vst.128.ip q4, a9, -16
    /* Filter: each output is a dot product over both groups, rounded. */
    ee.vld.128.ip q2, a8, 16
    ee.vld.128.ip q5, a8, 16
    ee.ld.accx.ip a10, 0
    ee.vmulas.s16.accx q0, q2
    ee.vmulas.s16.accx q3, q5
    ee.srs.accx a2, a14, 0
    ee.ld.accx.ip a10, 0
    ee.vmulas.s16.accx q1, q2
    ee.vmulas.s16.accx q4, q5
    ee.srs.accx a4, a14, 0
    ee.vld.128.ip q2, a8, 16
    ee.vld.128.ip q5, a8, -48
    ee.ld.accx.ip a10, 0
    ee.vmulas.s16.accx q0, q2
    ee.vmulas.s16.accx q3, q5
    ee.srs.accx a5, a14, 0
    ee.ld.accx.ip a10, 0
    ee.vmulas.s16.accx q1, q2
    ee.vmulas.s16.accx q4, q5
    ee.srs.accx a15, a14, 0
    /* Pack each output's I and Q to 10 bits each. */
    clamps a2, a2, 9
    clamps a4, a4, 9
    clamps a5, a5, 9
    clamps a15, a15, 9
    extui a2, a2, 0, 10
    slli a4, a4, 22
    srli a4, a4, 12
    or a2, a2, a4
    s32i a2, a6, 0
    extui a5, a5, 0, 10
    slli a15, a15, 22
    srli a15, a15, 12
    or a5, a5, a15
    s32i a5, a6, 4
    addi a6, a6, 8
1:
2:
    retw.n
    .size narrow_run, . - narrow_run
