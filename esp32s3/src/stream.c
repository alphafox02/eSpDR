/*
 * Burst-gated IQ streaming over USB serial.
 *
 * USB Full Speed carries about 1 MB/s, while 16 Msps of IQ is 40 MB/s even
 * packed, so the band cannot be streamed whole. The dump engine runs
 * continuously and only the bursts are forwarded. The work is split between
 * the cores so the time-critical part never waits on the bursty part:
 *
 * Core 1 manages the capture banks. While a bank is selected for the writer,
 * the CPU cannot see its contents (reads return a stale latched word), so
 * the writer rotates through banks 0, 1 and 3 and the CPU reads only banks
 * the writer has left, which always holds the two latest segments. Before a
 * bank is handed to the writer, sentinels are planted around the points
 * where its segment can start and end; after it is left, the first and last
 * written pairs are found exactly. A segment that does not start where the
 * previous one ended, or whose ends cannot be found, is passed to core 0 as
 * a hole. Core 1 also feeds USB: the endpoint has a single 64-byte buffer,
 * and a refill that misses the host's next request costs a NAK and usually
 * the rest of the frame, so the buffer is refilled from this tight loop
 * rather than between detection blocks.
 *
 * Core 0 measures power in 128-pair blocks (every eighth pair) and tracks
 * the noise floor while the band is quiet. A record opens when a block rises
 * 6 dB above the floor, a few blocks early so preambles are kept, and closes
 * once the power has stayed within 3 dB of the floor for the hold time.
 * Optionally, bursts whose envelope fluctuates like OFDM (Wi-Fi) are dropped,
 * keeping constant-envelope ones such as GFSK (Bluetooth). Kept bursts are
 * packed into a queue in bank 2, which core 1 drains to USB. The writer
 * fills bank 3, whose top holds the ROM's working memory, so that memory is
 * saved before streaming and restored afterwards, and no ROM routine runs in
 * between.
 *
 * Optionally each burst is cut down to its own channel before it is queued,
 * a quarter of the data: its frequency is measured over its first 32 us,
 * rounded to whole MHz, and the burst from its lead-in on is mixed down,
 * filtered and decimated to 4 Msps with the vector extension (narrow.h).
 * Bluetooth keeps most of its power in that channel and Wi-Fi does not,
 * which makes a sharper Wi-Fi test than the envelope.
 */
#include "stream.h"

#include <stdbool.h>

#include "board.h"
#include "capture.h"
#include "control.h"
#include "link.h"
#include "narrow.h"
#include "platform.h"
#include "radio.h"
#include "soc/usb_serial_jtag_reg.h"

#define RING_MASK (LINK_RING_PAIRS - 1u)
#define SENTINEL 0xA5C33C5Au      /* the dump format's top bits never take this value */
#define SEG_PAIRS 3584u           /* pairs per writer segment, 224 us at 16 Msps */
#define END_GUARD 1024u           /* sentinels past each segment's earliest end */
#define START_GUARD 1024u         /* sentinels around each segment's earliest start */
#define START_LEAD 64u            /* of which this many precede the earliest start */
#define SWITCH_SETTLE_CYCLES 960u /* pairs in the ADC pipeline land within this */
#define WRITER_BANKS 3u
/* The writer rotates through banks 0, 1 and 3, leaving all of bank 2 for the
 * output queue. Bank 3's top holds the ROM's working memory, which is saved
 * and restored around a stream as the FPGA capture path does. */
static const unsigned writer_bank[WRITER_BANKS] = {0, 1, 3};
#define BACKLOG_LIMIT (2u * SEG_PAIRS - 512u) /* unread pairs before data is at risk */
#define CLOSED_SEGMENTS 8u        /* closed segments kept for lookups */

#define BLOCK_PAIRS 128u
#define POWER_STRIDE 8u           /* power from every eighth pair */
#define FLOOR_INIT_BLOCKS 32u
#define PRE_BLOCKS 4u             /* 32 us of lead-in at 16 Msps */
#define HOLD_BLOCKS 4u            /* 32 us below the off threshold ends a burst */
#define CLASSIFY_BLOCKS 4u        /* decide on the envelope after 32 us */
#define ON_RATIO 4.0f             /* +6 dB */
#define OFF_RATIO 2.0f            /* +3 dB */
#define FLOOR_WEIGHT (1.0f / 32.0f)
#define ENVELOPE_CV2_MAX 0.36f    /* power coefficient of variation 0.6, squared */
#define DEFAULT_MAX_PAIRS (8u * 1024u)    /* 512 us; fits the queue with room to spare */
#define NARROW_MAX_PAIRS (48u * 1024u)    /* 3 ms in, 30 KB out: a whole 3-DH5 packet */
#define ESTIMATE_BLOCKS 4u                /* 32 us of signal decide a burst's channel */
#define NARROW_CHUNK_GROUPS 32u            /* groups of 8 pairs filtered per call */
#define ROTATION_STRIDE 4u                /* frequency from every fourth pair and the next */
/* A Bluetooth burst keeps most of its power within its channel's filter;
 * Wi-Fi spread over the window keeps about a fifth. */
#define IN_CHANNEL_MIN 0.5f
#define STATUS_INTERVAL_PAIRS 4000000u /* 250 ms at 16 Msps */
#define STATUS_WORDS 8u
#define HEADER_BYTES 24u
#define USB_PACKET 64u

#define QUEUE_BASE (CAPTURE_BANK_BASE + 2u * CAPTURE_BANK_BYTES) /* bank 2 */
#define QUEUE_BYTES CAPTURE_BANK_BYTES
#define QUEUE_MASK (QUEUE_BYTES - 1u)

_Static_assert((QUEUE_BYTES & QUEUE_MASK) == 0, "queue size must be a power of two");
/* A bank is reused every third segment. Its next segment and that segment's
 * guard must not reach the previous segment's pairs, with room for late
 * switches. */
_Static_assert(4u * SEG_PAIRS + END_GUARD + 1024u <= LINK_RING_PAIRS, "segments overlap within a bank");

/* Pair n (counted from the stream's origin) sits at ring position
 * (origin + n) & RING_MASK of the bank its segment was written to. */
typedef struct {
    uint64_t start, end;
    uint32_t bank;
    uint32_t unknown_end; /* the writer overran the guard; end is approximate */
    uint32_t gap_before;  /* pairs between the previous segment's end and this start */
} segment;

/* ---- shared between the cores ------------------------------------------ */

volatile uint32_t stream_core1_request; /* bumped by core 0 to start core 1 */
static volatile uint32_t sw_stop, sw_ready, sw_done;
static volatile uint32_t sw_control;     /* dump control word, from core 0 */
static volatile uint32_t sw_origin;
static volatile uint32_t sw_first_valid_lo, sw_first_valid_hi;
static volatile uint32_t sw_closed_count;
static segment sw_closed[CLOSED_SEGMENTS];
/* Segments whose ends could not be found, or that did not start where the
 * previous one ended. */
static volatile uint32_t sw_discontinuities;
static volatile uint32_t sw_writer_stopped, sw_exit;

/* The output queue: core 0 produces, core 1 sends. Cursors are running byte
 * counts; q_commit (end of the last complete record) is written only by
 * core 0 and q_tail (next byte to send) only by core 1. */
static uint8_t *const queue = (uint8_t *)QUEUE_BASE;
static volatile uint32_t q_commit;
static volatile uint32_t q_tail;

static inline volatile uint32_t *bank(unsigned b)
{
    return (volatile uint32_t *)(CAPTURE_BANK_BASE + b * CAPTURE_BANK_BYTES);
}

/* ---- core 1: capture banks ----------------------------------------------- */

static uint32_t c1_origin;
static uint64_t c1_prep_base[CAPTURE_BANKS]; /* earliest possible end of each bank's next segment */
static uint64_t c1_start_base[CAPTURE_BANKS]; /* earliest possible start of each bank's next segment */

static void CORE1_CODE c1_spin(uint32_t cycles)
{
    uint32_t start = cpu_cycles();
    while (cpu_cycles() - start < cycles) {
    }
}

static void CORE1_CODE c1_select(unsigned b)
{
    REG(DUMP_BANK_SELECT_REG) = (REG(DUMP_BANK_SELECT_REG) & ~15u) | (1u << b);
    memory_barrier();
}

static uint64_t CORE1_CODE c1_written(uint32_t *last_index, uint64_t written)
{
    uint32_t index = REG(DUMP_WRITE_INDEX_REG) & RING_MASK;
    written += (index - *last_index) & RING_MASK;
    *last_index = index;
    return written;
}

/* Plants sentinels over pairs [from, from + count) of bank b (not selected). */
static void CORE1_CODE c1_fill(unsigned b, uint64_t from, uint32_t count)
{
    uint32_t *p = (uint32_t *)(CAPTURE_BANK_BASE + b * CAPTURE_BANK_BYTES);
    uint32_t pos = (c1_origin + (uint32_t)from) & RING_MASK;
    uint32_t first = LINK_RING_PAIRS - pos;
    if (first > count)
        first = count;
    for (uint32_t k = 0; k < first; k++)
        p[pos + k] = SENTINEL;
    for (uint32_t k = 0; k < count - first; k++)
        p[k] = SENTINEL;
}

/* Readies bank b (not selected) for a segment that cannot start before
 * `start` or end before `end`: sentinels around both, so the first and last
 * pairs the writer puts there can be found exactly afterwards. */
static void CORE1_CODE c1_prepare(unsigned b, uint64_t start, uint64_t end)
{
    c1_fill(b, start - START_LEAD, START_GUARD);
    c1_fill(b, end, END_GUARD);
    memory_barrier();
    c1_start_base[b] = start;
    c1_prep_base[b] = end;
}

/* First pair the writer put in bank b's start zone; 0 if the zone holds no
 * written pair or its first pair was already written (start not bracketed). */
static uint64_t CORE1_CODE c1_find_start(unsigned b)
{
    const volatile uint32_t *p = bank(b);
    uint64_t base = c1_start_base[b] - START_LEAD;
    uint32_t at = c1_origin + (uint32_t)base;
    if (p[at & RING_MASK] != SENTINEL || p[(at + START_GUARD - 1u) & RING_MASK] == SENTINEL)
        return 0;
    uint32_t lo = 0, hi = START_GUARD - 1u;
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2u;
        if (p[(at + mid) & RING_MASK] == SENTINEL)
            lo = mid + 1u;
        else
            hi = mid;
    }
    return base + lo;
}

/* First pair at or after the bank's prepared base still holding a sentinel:
 * the exact end of the segment just written to bank b. 0 if the writer ran
 * past the guard. */
static uint64_t CORE1_CODE c1_find_end(unsigned b)
{
    const volatile uint32_t *p = bank(b);
    uint64_t base = c1_prep_base[b];
    uint32_t at = c1_origin + (uint32_t)base;
    if (p[(at + END_GUARD - 1u) & RING_MASK] != SENTINEL)
        return 0;
    uint32_t lo = 0, hi = END_GUARD - 1u;
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2u;
        if (p[(at + mid) & RING_MASK] == SENTINEL)
            hi = mid;
        else
            lo = mid + 1u;
    }
    return base + lo;
}

static void CORE1_CODE c1_publish(uint64_t start, uint64_t end, unsigned b, bool unknown_end, uint32_t gap)
{
    uint32_t n = sw_closed_count;
    segment *s = &sw_closed[n % CLOSED_SEGMENTS];
    s->start = start;
    s->end = end;
    s->bank = b;
    s->unknown_end = unknown_end;
    s->gap_before = gap;
    memory_barrier();
    sw_closed_count = n + 1u;
    memory_barrier();
}

/* ---- core 1: USB --------------------------------------------------------- */

static unsigned c1_in_packet;
static uint32_t c1_usb_sent;
/* True when the last packet handed to USB was a full one: once the FIFO then
 * reports free space, the host has taken that packet and the FIFO is empty,
 * so the next 64 bytes can be written without checking each one. */
static bool c1_fifo_empty;

/* Sends at most one USB packet of committed queue data, without waiting. */
static void CORE1_CODE c1_usb(void)
{
    uint32_t tail = q_tail;
    uint32_t avail = q_commit - tail;
    if (!avail || !(REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE))
        return;
    if (c1_fifo_empty && c1_in_packet == 0 && avail >= USB_PACKET) {
        for (unsigned i = 0; i < USB_PACKET; i++)
            REG(USB_SERIAL_JTAG_EP1_REG) = queue[(tail + i) & QUEUE_MASK];
        REG(USB_SERIAL_JTAG_EP1_CONF_REG) = USB_SERIAL_JTAG_WR_DONE;
        q_tail = tail + USB_PACKET;
        c1_usb_sent++;
        return;
    }
    while (tail != q_commit &&
           (REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE)) {
        REG(USB_SERIAL_JTAG_EP1_REG) = queue[tail++ & QUEUE_MASK];
        if (++c1_in_packet == USB_PACKET) {
            REG(USB_SERIAL_JTAG_EP1_CONF_REG) = USB_SERIAL_JTAG_WR_DONE;
            c1_in_packet = 0;
            c1_fifo_empty = true;
            q_tail = tail;
            return;
        }
    }
    if (c1_in_packet && tail == q_commit) {
        REG(USB_SERIAL_JTAG_EP1_CONF_REG) = USB_SERIAL_JTAG_WR_DONE;
        c1_in_packet = 0;
        c1_fifo_empty = false;
    }
    q_tail = tail;
}

void CORE1_CODE stream_core1(void)
{
    uint32_t control = sw_control;
    c1_in_packet = 0;
    c1_fifo_empty = false;
    sw_closed_count = 0;
    sw_discontinuities = 0;

    /* Start the writer in bank 0. Its first segment is discarded: it was not
     * prepared with sentinels, and its start depends on how the writer
     * resumes. Pairs are counted from the index seen just after starting. */
    REG(DUMP_CTRL_REG) = control;
    c1_select(0);
    REG(DUMP_CTRL_REG) = control | DUMP_CTRL_RUN;
    memory_barrier();
    uint32_t last_index = REG(DUMP_WRITE_INDEX_REG) & RING_MASK;
    /* Count pairs from a multiple of 8 in the ring, so that every group of
     * eight pairs starting at a multiple of 8 is aligned for vector loads. */
    c1_origin = last_index & ~7u;
    uint64_t written = last_index - c1_origin;

    /* Move to the second writer bank at a known point. Each bank is prepared
     * just before the writer enters it, with sentinels where its segment can
     * start (no earlier than the threshold at which the previous bank is left)
     * and end (no earlier than one segment after that). */
    written = c1_written(&last_index, written);
    uint64_t threshold = written;
    c1_prepare(writer_bank[1], threshold, threshold + SEG_PAIRS);
    c1_select(writer_bank[1]);
    unsigned slot = 1;
    c1_spin(SWITCH_SETTLE_CYCLES);
    threshold += SEG_PAIRS; /* leave the second bank here */
    c1_prepare(writer_bank[2], threshold, threshold + SEG_PAIRS);
    /* Pairs before this point may have landed in either bank. */
    written = c1_written(&last_index, written);
    uint64_t first_valid = (written + BLOCK_PAIRS) & ~(uint64_t)(BLOCK_PAIRS - 1u);
    sw_origin = c1_origin;
    sw_first_valid_lo = (uint32_t)first_valid;
    sw_first_valid_hi = (uint32_t)(first_valid >> 32);
    memory_barrier();
    sw_ready = 1;
    memory_barrier();

    uint64_t prev_end = 0; /* end of the last published segment; 0 before the first */
    while (!sw_stop) {
        c1_usb();
        written = c1_written(&last_index, written);
        if (written < threshold)
            continue;

        /* Hand the writer the next bank, then find exactly where the writer
         * started and stopped in the one it left. */
        uint64_t switch_written = written;
        unsigned left = writer_bank[slot];
        slot = (slot + 1u) % WRITER_BANKS;
        c1_select(writer_bank[slot]);
        c1_spin(SWITCH_SETTLE_CYCLES);
        uint64_t end = c1_find_end(left);
        uint64_t start = c1_find_start(left);
        bool unknown = !end || !start || start >= end;
        if (unknown) {
            /* Resynchronise on the writer instead of guessing: the segment
             * becomes a hole and the next one is scheduled from now. */
            start = end = switch_written;
        }
        uint32_t gap = 0;
        if (prev_end && start != prev_end)
            gap = start > prev_end ? (uint32_t)(start - prev_end) : 0xFFFFFFFFu;
        if (unknown || gap)
            sw_discontinuities = sw_discontinuities + 1u;
        c1_publish(start, end, left, unknown, gap);
        prev_end = end;

        /* The bank now being written is left one segment after its start,
         * which is this end; the bank after it starts no earlier than that. */
        threshold = end + SEG_PAIRS;
        c1_prepare(writer_bank[(slot + 1u) % WRITER_BANKS], threshold, threshold + SEG_PAIRS);
    }

    REG(DUMP_CTRL_REG) = control;
    memory_barrier();
    REG(DUMP_BANK_SELECT_REG) &= ~15u;
    memory_barrier();
    sw_writer_stopped = 1;
    memory_barrier();

    /* Keep sending until core 0 has queued its last record and seen it out. */
    while (!sw_exit)
        c1_usb();
    sw_done = 1;
    memory_barrier();
}

/* ---- core 0: segment lookups --------------------------------------------- */

static uint32_t origin;
/* Stretches of the stream that no segment holds: between a segment's end and
 * the next one's start, or a whole segment whose ends were not found. */
#define HOLES 16u
static uint64_t hole_start[HOLES], hole_end[HOLES];
static uint32_t hole_head, hole_tail;
static uint32_t seen_count; /* closed segments core 0 has taken in */
static segment lookup;      /* cached segment for run_at() */

/* Points at pair n, which must lie in a recently closed segment, and sets
 * *avail to how many pairs from n on are contiguous in memory: within that
 * segment and before the ring wraps. */
static inline const volatile uint32_t *run_at(uint64_t n, uint32_t *avail)
{
    if (n < lookup.start || n >= lookup.end) {
        for (unsigned i = 1; i <= CLOSED_SEGMENTS && i <= seen_count; i++) {
            const segment *s = &sw_closed[(seen_count - i) % CLOSED_SEGMENTS];
            if (n >= s->start && n < s->end) {
                lookup = *s;
                break;
            }
        }
        if (n < lookup.start || n >= lookup.end) {
            *avail = 0;
            return bank(0);
        }
    }
    uint32_t pos = (origin + (uint32_t)n) & RING_MASK;
    uint32_t in_segment = (uint32_t)(lookup.end - n);
    uint32_t to_wrap = LINK_RING_PAIRS - pos;
    *avail = in_segment < to_wrap ? in_segment : to_wrap;
    return bank(lookup.bank) + pos;
}

/* ---- queue -------------------------------------------------------------- */

static uint32_t q_head; /* next byte core 0 writes */

/* Record being built. */
static uint32_t rec_start; /* q_head at the record's header */
static uint32_t rec_type;
static uint32_t rec_pairs;
static uint32_t rec_carry; /* first pair of an incomplete two-pair group */
static bool rec_has_carry;
static uint32_t rec_sum;   /* sum of the 20-bit pairs, or of status words */
static bool rec_open;

static uint32_t sequence;
static struct {
    uint32_t sent, rejected, dropped, truncated, overruns, abandoned;
} counters;

static inline uint32_t queue_free(void) { return QUEUE_BYTES - (q_head - q_tail); }

static inline void put_byte(uint8_t b) { queue[q_head++ & QUEUE_MASK] = b; }

static inline void put_u32_at(uint32_t at, uint32_t v)
{
    for (unsigned i = 0; i < 4; i++)
        queue[(at + i) & QUEUE_MASK] = (uint8_t)(v >> (8 * i));
}

/* Starts a record if the queue has room for its header, `payload` bytes and
 * the trailer. */
static bool record_begin(uint32_t type, uint64_t start, uint32_t payload)
{
    if (queue_free() < HEADER_BYTES + payload + 4u)
        return false;
    rec_start = q_head;
    rec_type = type;
    put_u32_at(q_head, STREAM_MAGIC);
    put_u32_at(q_head + 4, type);
    put_u32_at(q_head + 8, sequence);
    put_u32_at(q_head + 12, (uint32_t)start);
    put_u32_at(q_head + 16, (uint32_t)(start >> 32));
    put_u32_at(q_head + 20, 0);
    q_head += HEADER_BYTES;
    rec_pairs = 0;
    rec_has_carry = false;
    rec_sum = 0;
    rec_open = true;
    return true;
}

static void record_abort(void)
{
    q_head = rec_start;
    rec_open = false;
}

static void record_end(uint32_t flags, uint32_t length)
{
    put_u32_at(rec_start + 4, rec_type | flags << 16);
    put_u32_at(rec_start + 20, length);
    put_u32_at(q_head, rec_sum);
    q_head += 4;
    memory_barrier();
    q_commit = q_head;
    rec_open = false;
    sequence++;
}

/* Packs pairs a and b into 5 bytes at queue offset `at`, wrapping. */
static inline void put_group_wrapped(uint32_t at, uint32_t a, uint32_t b)
{
    uint32_t lo = a | b << 20;
    queue[at & QUEUE_MASK] = (uint8_t)lo;
    queue[(at + 1u) & QUEUE_MASK] = (uint8_t)(lo >> 8);
    queue[(at + 2u) & QUEUE_MASK] = (uint8_t)(lo >> 16);
    queue[(at + 3u) & QUEUE_MASK] = (uint8_t)(lo >> 24);
    queue[(at + 4u) & QUEUE_MASK] = (uint8_t)(b >> 12);
}

enum copy_result { COPY_OK, COPY_FULL, COPY_HOLE };

/* Appends pairs [from, from + count) to the open burst record. COPY_FULL if
 * the queue has no room, COPY_HOLE if a pair is missing from the stream. Pairs are packed two at a time straight into the
 * queue; only a group that straddles the queue's end takes the masked path.
 * The cursor, carry and sum live in locals: byte stores may alias any global,
 * which would otherwise force them to be reloaded around every byte. */
static enum copy_result record_pairs(uint64_t from, uint32_t count)
{
    if (queue_free() < (count / 2u + 1u) * 5u + 4u)
        return COPY_FULL;
    uint32_t head = q_head, sum = rec_sum, carry = rec_carry;
    bool has_carry = rec_has_carry;
    for (uint32_t k = 0; k < count;) {
        uint32_t avail;
        const volatile uint32_t *p = run_at(from + k, &avail);
        if (!avail)
            return COPY_HOLE; /* a hole: the record cannot be completed */
        if (avail > count - k)
            avail = count - k;
        uint32_t j = 0;
        if (has_carry) {
            if (p[0] == SENTINEL)
                return COPY_HOLE;
            uint32_t b = p[0] & 0xFFFFFu;
            sum += b;
            put_group_wrapped(head, carry, b);
            head += 5u;
            has_carry = false;
            j = 1;
        }
        uint32_t groups = (avail - j) / 2u;
        uint32_t offset = head & QUEUE_MASK;
        if (offset + groups * 5u <= QUEUE_BYTES) {
            uint8_t *d = queue + offset;
            for (uint32_t g = 0; g < groups; g++, j += 2u, d += 5) {
                uint32_t a = p[j] & 0xFFFFFu;
                uint32_t b = p[j + 1u] & 0xFFFFFu;
                if (p[j] == SENTINEL || p[j + 1u] == SENTINEL)
                    return COPY_HOLE; /* the writer skipped this pair: not signal */
                sum += a + b;
                uint32_t lo = a | b << 20;
                d[0] = (uint8_t)lo;
                d[1] = (uint8_t)(lo >> 8);
                d[2] = (uint8_t)(lo >> 16);
                d[3] = (uint8_t)(lo >> 24);
                d[4] = (uint8_t)(b >> 12);
            }
        } else {
            for (uint32_t g = 0; g < groups; g++, j += 2u) {
                if (p[j] == SENTINEL || p[j + 1u] == SENTINEL)
                    return COPY_HOLE;
                uint32_t a = p[j] & 0xFFFFFu;
                uint32_t b = p[j + 1u] & 0xFFFFFu;
                sum += a + b;
                put_group_wrapped(head + g * 5u, a, b);
            }
        }
        head += groups * 5u;
        if (j < avail) {
            if (p[j] == SENTINEL)
                return COPY_HOLE;
            carry = p[j] & 0xFFFFFu;
            sum += carry;
            has_carry = true;
        }
        k += avail;
    }
    q_head = head;
    rec_sum = sum;
    rec_carry = carry;
    rec_has_carry = has_carry;
    rec_pairs += count;
    return COPY_OK;
}

/* Pads a trailing odd pair with a zero pair and closes the burst record. */
static void record_burst_end(uint32_t flags)
{
    if (rec_has_carry) {
        put_byte((uint8_t)rec_carry);
        put_byte((uint8_t)(rec_carry >> 8));
        put_byte((uint8_t)(rec_carry >> 16));
        put_byte(0);
        put_byte(0);
        rec_has_carry = false;
    }
    record_end(flags, rec_pairs);
}

/* ---- channelized bursts --------------------------------------------------- */

/* A narrow record carries one channel of a burst, cut by narrow_run(): the
 * pairs are mixed down by the burst's offset k, filtered and kept at 4 Msps.
 * Output j is centred on pair start + 4j - 2.5. */
static narrow_state narrow;

/* cos and sin of 2 pi m / 16, for scoring channels. */
static const int16_t unit_cos[16] = {16384, 15137, 11585, 6270, 0, -6270, -11585, -15137,
                                     -16384, -15137, -11585, -6270, 0, 6270, 11585, 15137};

/* Sum of x[n+1] * conj(x[n]) over every fourth pair n of the block at pair
 * n, whose phase is the mean frequency of what is in it. */
static void block_rotation(uint64_t n, float *re, float *im)
{
    int32_t sr = 0, si = 0;
    for (uint32_t k = 0; k < BLOCK_PAIRS;) {
        uint32_t avail;
        const volatile uint32_t *p = run_at(n + k, &avail);
        if (!avail)
            break;
        uint32_t stop = k + avail < BLOCK_PAIRS ? k + avail : BLOCK_PAIRS;
        for (uint32_t j = 0; k + 1u < stop; k += ROTATION_STRIDE, j += ROTATION_STRIDE) {
            uint32_t w0 = p[j], w1 = p[j + 1u];
            int32_t i0 = (int32_t)(w0 << 22) >> 22, q0 = (int32_t)(w0 << 12) >> 22;
            int32_t i1 = (int32_t)(w1 << 22) >> 22, q1 = (int32_t)(w1 << 12) >> 22;
            sr += i1 * i0 + q1 * q0;
            si += q1 * i0 - i1 * q0;
        }
        k = stop;
    }
    *re += (float)sr;
    *im += (float)si;
}

/* The whole-MHz offset in -7..7 nearest the rotation's phase. */
static int32_t nearest_channel(float re, float im)
{
    int32_t best = -7;
    float best_score = 0.0f;
    for (int32_t k = -7; k <= 7; k++) {
        float score = re * (float)unit_cos[k & 15] + im * (float)unit_cos[(k + 12) & 15];
        if (k == -7 || score > best_score) {
            best = k;
            best_score = score;
        }
    }
    return best;
}

static bool narrow_begin(uint64_t start, int32_t k)
{
    if (!record_begin(STREAM_NARROW, start, 0))
        return false;
    narrow_reset(&narrow, k);
    return true;
}

/* Feeds pairs [from, from + count) through the channel filter into the open
 * narrow record; both multiples of 8. Results as for record_pairs(). Each
 * group of eight pairs gives two outputs, one 5-byte group of the record.
 * If `power` is given, the outputs' |IQ|^2 is added to it. */
static enum copy_result record_narrow(uint64_t from, uint32_t count, uint32_t *power)
{
    if (queue_free() < count / 8u * 5u + 4u)
        return COPY_FULL;
    static uint32_t gather[8] __attribute__((aligned(16)));
    uint32_t out[2 * NARROW_CHUNK_GROUPS];
    uint32_t head = q_head, sum = rec_sum;
    for (uint32_t k = 0; k < count;) {
        uint32_t avail;
        const volatile uint32_t *p = run_at(from + k, &avail);
        if (!avail)
            return COPY_HOLE;
        uint32_t groups = (avail < count - k ? avail : count - k) / 8u;
        if (groups > NARROW_CHUNK_GROUPS)
            groups = NARROW_CHUNK_GROUPS;
        if (groups == 0) {
            /* The group straddles the end of a segment: collect it. */
            for (uint32_t j = 0; j < 8u; j++) {
                const volatile uint32_t *q = run_at(from + k + j, &avail);
                if (!avail || *q == SENTINEL)
                    return COPY_HOLE;
                gather[j] = *q;
            }
            p = gather;
            groups = 1;
        } else if (p[0] == SENTINEL || p[groups * 8u - 1u] == SENTINEL) {
            return COPY_HOLE;
        }
        narrow_run(&narrow, p, (uint32_t)(from + k), groups, out);
        if (power) {
            uint32_t e = *power;
            for (uint32_t j = 0; j < 2u * groups; j++) {
                int32_t i = (int32_t)(out[j] << 22) >> 22, q = (int32_t)(out[j] << 12) >> 22;
                e += (uint32_t)(i * i + q * q);
            }
            *power = e;
        }
        uint32_t offset = head & QUEUE_MASK;
        if (offset + groups * 5u <= QUEUE_BYTES) {
            uint8_t *d = queue + offset;
            for (uint32_t g = 0; g < groups; g++, d += 5) {
                uint32_t a = out[2u * g], b = out[2u * g + 1u];
                sum += a + b;
                uint32_t lo = a | b << 20;
                d[0] = (uint8_t)lo;
                d[1] = (uint8_t)(lo >> 8);
                d[2] = (uint8_t)(lo >> 16);
                d[3] = (uint8_t)(lo >> 24);
                d[4] = (uint8_t)(b >> 12);
            }
        } else {
            for (uint32_t g = 0; g < groups; g++) {
                uint32_t a = out[2u * g], b = out[2u * g + 1u];
                sum += a + b;
                put_group_wrapped(head + g * 5u, a, b);
            }
        }
        head += groups * 5u;
        k += groups * 8u;
    }
    q_head = head;
    rec_sum = sum;
    rec_pairs += count / 4u;
    return COPY_OK;
}

/* ---- USB ---------------------------------------------------------------- */

/* ---- core 0: detection --------------------------------------------------- */

/* Mean power of the block starting at pair n, from every eighth pair. */
static inline float block_power(uint64_t n)
{
    uint32_t sum = 0;
    for (uint32_t k = 0; k < BLOCK_PAIRS;) {
        uint32_t avail;
        const volatile uint32_t *p = run_at(n + k, &avail);
        if (!avail)
            break; /* a hole: score only what was present */
        uint32_t stop = k + avail < BLOCK_PAIRS ? k + avail : BLOCK_PAIRS;
        for (uint32_t j = 0; k < stop; k += POWER_STRIDE, j += POWER_STRIDE) {
            uint32_t w = p[j];
            int32_t i = (int32_t)(w << 22) >> 22;
            int32_t q = (int32_t)(w << 12) >> 22;
            sum += (uint32_t)(i * i + q * q);
        }
    }
    return (float)sum * ((float)POWER_STRIDE / (float)BLOCK_PAIRS);
}

static void put_status(uint64_t now, float floor)
{
    if (rec_open || !record_begin(STREAM_STATUS, now, STATUS_WORDS * 4u))
        return;
    union {
        float f;
        uint32_t u;
    } bits = {.f = floor};
    uint32_t words[STATUS_WORDS] = {
        bits.u, counters.sent, counters.rejected, counters.dropped,
        counters.truncated, counters.overruns, sw_discontinuities + counters.abandoned, q_head - q_tail,
    };
    for (unsigned i = 0; i < STATUS_WORDS; i++) {
        rec_sum += words[i];
        for (unsigned b = 0; b < 4; b++)
            put_byte((uint8_t)(words[i] >> (8 * b)));
    }
    record_end(0, 0);
}

unsigned stream_run(unsigned arg)
{
    if (radio_stat(ESP_STAT_RATE) != ESP_RATE_16M)
        return CTL_NOT_READY;
    bool reject_wideband = arg & STREAM_REJECT_WIDEBAND;
    bool channelize = arg & STREAM_CHANNELIZE;
    uint32_t max_pairs = ((arg >> STREAM_MAX_KPAIRS_SHIFT) & 0xFFu) * 1024u;
    if (!max_pairs)
        max_pairs = channelize ? NARROW_MAX_PAIRS : DEFAULT_MAX_PAIRS;

    q_head = 0;
    q_commit = 0;
    q_tail = 0;
    rec_open = false;
    sequence = 0;
    seen_count = 0;
    hole_head = hole_tail = 0;
    lookup = (segment){0, 0, 0, 0, 0};
    counters.sent = counters.rejected = counters.dropped = 0;
    counters.truncated = counters.overruns = 0;
    counters.abandoned = 0;

    /* The writer will fill bank 3, which holds the ROM's working memory: keep
     * a copy until it has stopped. No ROM routine runs while streaming. */
    capture_save_rom();

    /* Hand the capture banks to core 1 and wait until it is running. */
    sw_control = radio_dump_control();
    sw_stop = 0;
    sw_ready = 0;
    sw_done = 0;
    sw_writer_stopped = 0;
    sw_exit = 0;
    memory_barrier();
    stream_core1_request = stream_core1_request + 1u;
    memory_barrier();
    while (!sw_ready) {
    }
    memory_barrier();
    origin = sw_origin;
    const uint64_t first_valid = (uint64_t)sw_first_valid_hi << 32 | sw_first_valid_lo;
    uint64_t done = first_valid;
    uint64_t readable = 0; /* end of the last closed segment taken in */
    uint64_t next_status = done + STATUS_INTERVAL_PAIRS;

    float floor = 0.0f, env_sum = 0.0f, env_sum2 = 0.0f;
    unsigned init_blocks = 0, quiet = 0, classify = 0;
    /* PENDING: a channelized burst has begun, but its channel is still being
     * measured; its record opens once it is known. */
    enum { IDLE, PENDING, ACTIVE, IGNORE } state = IDLE;
    bool truncated = false;
    uint64_t burst_start = 0, burst_pairs = 0;
    float rot_re = 0.0f, rot_im = 0.0f;
    unsigned estimated = 0;

    for (uint32_t iteration = 0;; iteration++) {
        if ((iteration & 63u) == 0 && serial_rx_pending())
            break;

        /* Take in newly closed segments. One whose end is unknown cannot be
         * trusted: abandon any burst and resume after it. */
        uint32_t count = sw_closed_count;
        memory_barrier();
        while (seen_count < count) {
            const segment *s = &sw_closed[seen_count % CLOSED_SEGMENTS];
            seen_count++;
            uint64_t from = readable > s->start ? s->start : readable;
            uint64_t to = s->unknown_end ? s->end : s->start;
            if (readable && to > from && hole_head - hole_tail < HOLES) {
                hole_start[hole_head % HOLES] = from;
                hole_end[hole_head % HOLES] = to;
                hole_head++;
            }
            readable = s->end;
        }

        /* Step over a hole that the next block would reach: a burst must not
         * be stitched across missing samples. */
        while (hole_tail != hole_head && hole_end[hole_tail % HOLES] <= done)
            hole_tail++;
        if (hole_tail != hole_head && hole_start[hole_tail % HOLES] < done + BLOCK_PAIRS) {
            if (rec_open) {
                record_abort();
                counters.abandoned++;
            }
            state = IDLE;
            done = (hole_end[hole_tail % HOLES] + BLOCK_PAIRS - 1u) & ~(uint64_t)(BLOCK_PAIRS - 1u);
            hole_tail++;
            continue;
        }

        if (readable > done && readable - done > BACKLOG_LIMIT) {
            counters.overruns++;
            if (rec_open)
                record_abort();
            state = IDLE;
            done = (readable - SEG_PAIRS) & ~(uint64_t)(BLOCK_PAIRS - 1u);
        }

        if (done + BLOCK_PAIRS > readable) {
            /* Caught up: use the slack for USB and housekeeping. */
            if (done >= next_status) {
                put_status(done, floor);
                next_status = done + STATUS_INTERVAL_PAIRS;
            }
            continue;
        }

        float p = block_power(done);

        if (init_blocks < FLOOR_INIT_BLOCKS) {
            floor += p / (float)FLOOR_INIT_BLOCKS;
            init_blocks++;
        } else if (state == IDLE) {
            if (p > floor * ON_RATIO && floor > 0.0f) {
                uint64_t lead = PRE_BLOCKS * BLOCK_PAIRS;
                if (done - first_valid < lead)
                    lead = done - first_valid;
                burst_start = done - lead;
                burst_pairs = lead + BLOCK_PAIRS;
                enum copy_result r = COPY_FULL;
                if (channelize) {
                    rot_re = rot_im = 0.0f;
                    block_rotation(done, &rot_re, &rot_im);
                    estimated = 1;
                    r = COPY_OK;
                } else if (record_begin(STREAM_BURST, burst_start, 0)) {
                    r = record_pairs(burst_start, (uint32_t)burst_pairs);
                }
                if (r == COPY_OK && channelize) {
                    state = PENDING;
                } else if (r == COPY_OK) {
                    state = ACTIVE;
                } else {
                    if (rec_open)
                        record_abort();
                    if (r == COPY_HOLE)
                        counters.abandoned++;
                    else
                        counters.dropped++;
                    state = IGNORE;
                }
                quiet = 0;
                classify = 1;
                env_sum = p;
                env_sum2 = p * p;
                truncated = false;
            } else {
                floor += (p - floor) * FLOOR_WEIGHT;
            }
        } else {
            quiet = p < floor * OFF_RATIO ? quiet + 1 : 0;
            if ((state == ACTIVE || state == PENDING) && classify < CLASSIFY_BLOCKS) {
                env_sum += p;
                env_sum2 += p * p;
                if (++classify == CLASSIFY_BLOCKS && reject_wideband) {
                    float mean = env_sum / (float)CLASSIFY_BLOCKS;
                    float var = env_sum2 / (float)CLASSIFY_BLOCKS - mean * mean;
                    if (var > ENVELOPE_CV2_MAX * mean * mean) {
                        if (rec_open)
                            record_abort();
                        counters.rejected++;
                        state = IGNORE;
                    }
                }
            }
            if (state == PENDING) {
                /* Once enough of the burst has been seen, or it has ended,
                 * open its record and filter everything so far. */
                burst_pairs += BLOCK_PAIRS;
                if (estimated < ESTIMATE_BLOCKS && quiet == 0) {
                    block_rotation(done, &rot_re, &rot_im);
                    estimated++;
                }
                if (estimated >= ESTIMATE_BLOCKS || quiet >= HOLD_BLOCKS) {
                    /* The lead-in, then the blocks that were measured, whose
                     * power in the channel is compared with their power in
                     * the window. */
                    uint64_t measured = burst_pairs - (uint64_t)classify * BLOCK_PAIRS;
                    uint32_t in_channel = 0;
                    enum copy_result r = COPY_FULL;
                    if (narrow_begin(burst_start, nearest_channel(rot_re, rot_im))) {
                        r = record_narrow(burst_start, (uint32_t)measured, 0);
                        if (r == COPY_OK)
                            r = record_narrow(burst_start + measured, (uint32_t)(burst_pairs - measured), &in_channel);
                    }
                    float window = env_sum * (float)BLOCK_PAIRS / 4.0f;
                    if (r == COPY_OK && reject_wideband && (float)in_channel < IN_CHANNEL_MIN * window) {
                        record_abort();
                        counters.rejected++;
                        state = IGNORE;
                    } else if (r == COPY_OK) {
                        state = ACTIVE;
                    } else {
                        if (rec_open)
                            record_abort();
                        if (r == COPY_HOLE)
                            counters.abandoned++;
                        else
                            counters.dropped++;
                        state = IGNORE;
                    }
                }
            } else if (state == ACTIVE) {
                if (burst_pairs + BLOCK_PAIRS > max_pairs) {
                    truncated = true;
                } else {
                    burst_pairs += BLOCK_PAIRS;
                    enum copy_result r = channelize ? record_narrow(done, BLOCK_PAIRS, 0)
                                                    : record_pairs(done, BLOCK_PAIRS);
                    if (r != COPY_OK) {
                        record_abort();
                        if (r == COPY_HOLE)
                            counters.abandoned++;
                        else
                            counters.dropped++;
                        state = IGNORE;
                    }
                }
            }
            if (quiet >= HOLD_BLOCKS) {
                if (state == ACTIVE) {
                    uint32_t flags = truncated ? STREAM_TRUNCATED : 0;
                    if (channelize)
                        flags |= (uint32_t)(narrow.k + 8) << 8;
                    record_burst_end(flags);
                    counters.sent++;
                    if (truncated)
                        counters.truncated++;
                }
                state = IDLE;
            }
        }
        done += BLOCK_PAIRS;

    }

    /* Stop the writer (core 1), queue the END record, let core 1 send what is
     * queued, then release it. Give up on sending if the host stops reading. */
    sw_stop = 1;
    memory_barrier();
    while (!sw_writer_stopped) {
    }
    capture_restore_rom();
    if (rec_open)
        record_abort();
    if (record_begin(STREAM_END, done, 0))
        record_end(0, 0);
    uint32_t start = cpu_cycles();
    while (q_tail != q_commit && cpu_cycles() - start < 240000000u) {
    }
    sw_exit = 1;
    memory_barrier();
    while (!sw_done) {
    }
    while (serial_rx_pending())
        (void)serial_read();
    return CTL_OK;
}
