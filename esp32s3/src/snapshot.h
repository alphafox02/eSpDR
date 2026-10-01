/* One-shot IQ capture over USB serial, for hosts without the FPGA link. */
#pragma once

#include <stdint.h>

/* Pairs sent per snapshot: the full ring less a guard on each side of the
 * point where the writer stopped. */
#define SNAPSHOT_SEAM_GUARD 32u
#define SNAPSHOT_PAIRS (16384u - 2u * SNAPSHOT_SEAM_GUARD)

/*
 * Runs the dump engine into capture bank 0 long enough to overwrite its
 * 16384-pair ring, then stops it. Returns the number of non-zero words found
 * in the ring, so 0 means the writer did not run.
 */
unsigned snapshot_capture(void);

/*
 * Sends the last capture in time order: a 16-byte header ("SNAP", pair count,
 * rate selector, LO in Hz), SNAPSHOT_PAIRS raw 32-bit dump words, then a
 * CRC-32 of the words.
 */
void snapshot_send(void);

/* Channel filter self-test on the last snapshot: returns cycles taken by
 * narrow_test_run(); narrow_test_send() sends the 4096 input pairs and the
 * 1024 outputs as 32-bit words. */
uint32_t narrow_test_run(int32_t k);
void narrow_test_send(void);
