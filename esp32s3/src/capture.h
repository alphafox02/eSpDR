/* Continuous two-core acquisition and link transmission. */
#pragma once

#include <stdint.h>

/*
 * Streams IQ over both link lanes until `seconds` have elapsed (0: until the
 * host sends a byte). Blocks on core 0 and returns 0 or an ESP_FAIL_* code.
 * The link outputs must already be enabled.
 */
unsigned capture_run(unsigned seconds);

/*
 * Saves and restores the ROM's working memory at the top of capture bank 3,
 * for other code that lets the writer fill that bank. No ROM routine may run
 * in between.
 */
void capture_save_rom(void);
void capture_restore_rom(void);

/* Statistics of the last run, indexed by ESP_STAT_*. */
uint32_t capture_stat(unsigned index);
