/*
 * SPDX-FileCopyrightText: 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: 0BSD
 */

/* Burst-gated IQ streaming over USB serial, for hosts without the FPGA link. */
#pragma once

#include <stdint.h>

/* ESP_STREAM argument bits. */
#define STREAM_REJECT_WIDEBAND 0x0001u /* drop Wi-Fi: bursts with a fluctuating envelope (OFDM),
                                          or channelized, with little power in the channel */
#define STREAM_CHANNELIZE 0x0002u      /* send each burst cut to its channel at 4 Msps */
#define STREAM_TELEMETRY 0x0004u       /* send extended receiver telemetry */
#define STREAM_MAX_KPAIRS_SHIFT 8      /* bits 8..15: burst cap, units of 1024 pairs; 0 = default
                                          (512 us, or 3 ms channelized) */
#define STREAM_CHANNEL_MASK_SHIFT 16   /* bits 16..31 (sent with ESP_ARG_HIGH), channelized only:
                                          send only bursts whose channel offset k (-8..7 MHz) has
                                          bit k + 8 set, so receivers on one LO can share the
                                          positions; 0 = all */

/* Record header magic ("BRST") and types. */
#define STREAM_MAGIC 0x54535242u
#define STREAM_BURST 1  /* length = pairs; payload = pairs packed 2 per 5 bytes */
#define STREAM_STATUS 2 /* length = 0; payload = 8 32-bit words, see below */
#define STREAM_END 3    /* length = 0; no payload */
#define STREAM_NARROW 4 /* one channel of a burst at 4 Msps (STREAM_CHANNELIZE): length =
                           outputs, output j centred on pair start + 4j - 2.5; payload
                           packed as for a burst; flags bits 8..11 = the channel's offset
                           from the LO in MHz, + 8, LO-minus-RF like the samples */
#define STREAM_STATUS_V2 5 /* length = 16; first 8 words match STREAM_STATUS */
#define STREAM_TRUNCATED 0x0001u
/* Status-record flags. When set, the record's start is the pair count latched
 * just after the named USB SOF arrived. All devices below one host see the
 * same 11-bit frame number, giving several receivers a common time mark. */
#define STREAM_STATUS_USB_SOF 0x8000u
#define STREAM_STATUS_USB_FRAME 0x07FFu

/*
 * Runs the dump engine continuously into capture bank 0 and sends each burst
 * above the noise floor as a record over USB, until the host sends a byte.
 * Requires 16 Msps. Returns 0, or a CTL_* status if the stream cannot start.
 *
 * Record: 24-byte header (magic, type u16, flags u16, sequence u32, start
 * u64 in pairs since the stream began, length u32), the payload, then a
 * 32-bit check: the sum of the 20-bit pairs for a burst (including the zero
 * pair that pads an odd count), or of the words for a status record. Burst
 * payloads pack each pair's 10-bit I and Q into 20 bits, two pairs per 5
 * bytes, little-endian, as the dump engine produced them (LO-minus-RF
 * orientation). A status with STREAM_STATUS_USB_SOF has the USB frame in
 * flags and the pair count latched just after that SOF in start. Status words:
 * noise floor (float, mean |IQ|^2 per pair),
 * bursts sent, rejected (as wideband, or for a channel outside the mask), dropped for queue space, truncated,
 * blocks skipped because processing fell behind, stream discontinuities
 * (capture segments lost or not contiguous, plus bursts abandoned at them),
 * queue bytes in use.
 *
 * STREAM_STATUS_V2 appends interval counters for queue high-water, triggers,
 * trigger margin, rejection reasons and backlog.
 * See USB.md for the word layout.
 */
unsigned stream_run(unsigned arg);

/* Core 1's part of a stream: bumped by stream_run(), served by core1_main(). */
extern volatile uint32_t stream_core1_request;
void stream_core1(void);
