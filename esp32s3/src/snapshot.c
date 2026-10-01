/*
 * One-shot IQ capture over USB serial.
 *
 * The dump engine writes a circular 16384-pair ring in whichever capture bank
 * is selected. Here only bank 0 is used: start the writer, let the ring be
 * overwritten a few times, stop it, and send the ring oldest-first over the
 * USB Serial/JTAG port. No link GPIOs, second core or other banks are touched,
 * so this needs no FPGA. Bank 3, which overlaps the ROM's working memory, is
 * never selected.
 */
#include "snapshot.h"

#include "board.h"
#include "control.h"
#include "link.h"
#include "platform.h"
#include "radio.h"

#define SNAP_MAGIC 0x50414E53u /* "SNAP", little-endian */
#define RING_MASK (LINK_RING_PAIRS - 1u)
/* 16384 pairs take 1.024 ms at 16 Msps and 0.2 ms at 80 Msps. */
#define SNAPSHOT_RUN_US 3000u
#define SNAPSHOT_SETTLE_US 20u
#define SEND_CHUNK_PAIRS 1024u

_Static_assert(LINK_RING_PAIRS == 16384u, "snapshot.h assumes a 16384-pair ring");

static uint32_t stop_index;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, unsigned size)
{
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return crc;
}

unsigned snapshot_capture(void)
{
    volatile uint32_t *ring = (volatile uint32_t *)CAPTURE_BANK_BASE;
    uint32_t control = radio_dump_control();

    for (unsigned i = 0; i < LINK_RING_PAIRS; i++)
        ring[i] = 0;
    memory_barrier();

    REG(DUMP_CTRL_REG) = control;
    REG(DUMP_BANK_SELECT_REG) = (REG(DUMP_BANK_SELECT_REG) & ~15u) | 1u;
    memory_barrier();
    REG(DUMP_CTRL_REG) = control | DUMP_CTRL_RUN;
    memory_barrier();
    delay_us(SNAPSHOT_RUN_US);

    /* The writer advances a few pairs between this read and the stop; the
     * seam guard in snapshot_send() skips that region. */
    stop_index = REG(DUMP_WRITE_INDEX_REG) & RING_MASK;
    REG(DUMP_CTRL_REG) = control;
    memory_barrier();
    delay_us(SNAPSHOT_SETTLE_US);
    REG(DUMP_BANK_SELECT_REG) &= ~15u;
    memory_barrier();

    unsigned written = 0;
    for (unsigned i = 0; i < LINK_RING_PAIRS; i++)
        written += ring[i] != 0;
    return written;
}

void snapshot_send(void)
{
    const uint32_t *ring = (const uint32_t *)CAPTURE_BANK_BASE;
    uint32_t header[4] = {
        SNAP_MAGIC,
        SNAPSHOT_PAIRS,
        radio_stat(ESP_STAT_RATE),
        radio_stat(ESP_STAT_LO_HZ),
    };
    serial_write(header, sizeof(header));

    uint32_t crc = 0xFFFFFFFFu;
    unsigned index = (stop_index + SNAPSHOT_SEAM_GUARD) & RING_MASK;
    unsigned remaining = SNAPSHOT_PAIRS;
    while (remaining) {
        unsigned count = LINK_RING_PAIRS - index;
        if (count > remaining)
            count = remaining;
        if (count > SEND_CHUNK_PAIRS)
            count = SEND_CHUNK_PAIRS;
        const uint8_t *bytes = (const uint8_t *)(ring + index);
        crc = crc32_update(crc, bytes, count * 4u);
        serial_write(bytes, count * 4u);
        index = (index + count) & RING_MASK;
        remaining -= count;
    }
    crc = ~crc;
    serial_write(&crc, sizeof(crc));
}
