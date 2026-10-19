#include "gps.h"

#include "core_cm4.h"
#include "immap.h"
#include "io.h"
#include "log.h"
#include "serial_ns16550.h"
#include "phytimer.h"

#include "la9310.h"

#include <string.h>

#define GPS_UART ((NS16550_t)UART_BASEADDR)
#define GPS_UART_IRQ_PRIORITY 4

#define SERIAL_IER_ERBFI 0x01 // received data available interrupt
#define SERIAL_LSR_ERRORS 0x1E // overrun, parity, framing, break

// Most likely rates first. 9600 is the datasheet default, 115200 is seen on some revisions.
static const uint32_t candidate_baudrates[] = { 9600, 115200 };
#define BAUDRATE_LIST_COUNT (sizeof(candidate_baudrates) / sizeof(candidate_baudrates[0]))

// The module outputs NMEA once per second, so a window slightly over 1 s catches at least one
// burst even if the probe is not answered.
#define DETECT_WINDOW_MS 1500

// Divisor rounding error above which a rate is not attempted.
#define BAUDRATE_MAX_ERROR_PERMILLE 30

// While running, restart detection if only garbage has arrived for this long.
#define LINK_LOST_MS 3000
#define LINK_LOST_MIN_ERRORS 16

#define RX_BUFFER_SIZE 1024 // must be power of 2

// Validated sentences waiting to be read by the host. The module outputs well under 1 KB per second.
#define MESSAGE_BUFFER_SIZE 2048 // power of two

static struct {
    uint8_t data[RX_BUFFER_SIZE];
    volatile uint32_t head; // written by the ISR
    volatile uint32_t tail; // written by gps_process()
    volatile uint32_t overflows;
    volatile uint32_t line_errors;
} rx;

static struct {
    char data[MESSAGE_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t dropped;
} messages;

static gps_state_t gps;
static nmea_framer_t framer;
static uint32_t uart_clock;

static struct GPS_Baudrate_Detection {
    uint32_t state_start_tick;
    uint32_t detect_bytes;
    uint32_t detect_sentences;
    uint32_t detect_line_errors_start;
    uint32_t last_sentence_tick;
    uint32_t errors_since_sentence;
    uint8_t candidate_index;
    bool start_pending;
} baudrate_detection;

static inline uint32_t phytime_now(void)
{
    return ulPhyTimerComparatorRead(10);
}

static inline uint32_t ms_since(uint32_t phytime)
{
    const uint32_t diff = phytime_now() - phytime;
    const uint32_t ticks_per_second = PHYTIMER_CLOCK_IN_FROM_PLATFORM(g_la9310_current_platform_clk_hz);
    return diff / (ticks_per_second / 1000);
}

void gps_uart_irq_handler(void)
{
    uint8_t lsr;
    // Reading LSR clears the error flags, reading RBR clears the data interrupt.
    while ((lsr = IN_8(&GPS_UART->lsr)) & (SERIAL_LSR_DR | SERIAL_LSR_ERRORS))
    {
        if (lsr & SERIAL_LSR_ERRORS)
            ++rx.line_errors;
        if (!(lsr & SERIAL_LSR_DR))
            continue;

        const uint8_t byte = IN_8(&GPS_UART->rbr);
        const uint32_t head = rx.head;
        if (head - rx.tail >= RX_BUFFER_SIZE)
        {
            ++rx.overflows;
            continue;
        }
        rx.data[head & (RX_BUFFER_SIZE - 1)] = byte;
        rx.head = head + 1;
    }
}

static bool is_baudrate_achievable(uint32_t baudrate)
{
    // Same rounding as vSerialInit()
    const uint32_t divisor = (uart_clock + 8 * baudrate) / (16 * baudrate);
    if (divisor == 0 || divisor > 0xFFFF)
        return false;
    const uint32_t actual = uart_clock / (16 * divisor);
    const uint32_t error = actual > baudrate ? actual - baudrate : baudrate - actual;
    return error * 1000 / baudrate <= BAUDRATE_MAX_ERROR_PERMILLE;
}

static void uart_set_baudrate(uint32_t baudrate)
{
    // The ISR must not run while the divisor latch is selected, RBR is not accessible then.
    NVIC_DisableIRQ(IRQ_UART);
    vSerialInit(GPS_UART, baudrate, uart_clock);
    OUT_8(&GPS_UART->ier, SERIAL_IER_ERBFI);
    rx.tail = rx.head; // drop data received at the previous rate
    nmea_framer_reset(&framer);
    NVIC_ClearPendingIRQ(IRQ_UART);
    NVIC_EnableIRQ(IRQ_UART);
}

static void uart_write_sentence(const char *body, size_t length)
{
    static const char hex[] = "0123456789ABCDEF";
    const uint8_t checksum = nmea_checksum(body, length);
    const uint8_t trailer[] = { '*', hex[checksum >> 4], hex[checksum & 0xF], '\r', '\n' };

    vSerialWriteBlocking(GPS_UART, (const uint8_t *)"$", 1);
    vSerialWriteBlocking(GPS_UART, (const uint8_t *)body, length);
    vSerialWriteBlocking(GPS_UART, trailer, sizeof(trailer));
}

static void baudrate_start_candidate(void)
{
    // Skip rates the current UART clock can't generate accurately.
    while (baudrate_detection.candidate_index < BAUDRATE_LIST_COUNT &&
           !is_baudrate_achievable(candidate_baudrates[baudrate_detection.candidate_index]))
        ++baudrate_detection.candidate_index;

    if (baudrate_detection.candidate_index >= BAUDRATE_LIST_COUNT)
    {
        log_err("GPS_LINK_FAIL: no valid NMEA data at any baud rate." LOG_EOL);
        gps.link = GPS_LINK_FAIL;
        return;
    }

    const uint32_t baudrate = candidate_baudrates[baudrate_detection.candidate_index];
    log_dbg("GPS: trying %u baud" LOG_EOL, baudrate);
    uart_set_baudrate(baudrate);

    // $PMTK000 test command, the module replies with $PMTK001,0,3.
    static const char *gps_probe_sentence = "PMTK000";
    uart_write_sentence(gps_probe_sentence, strlen(gps_probe_sentence));

    baudrate_detection.detect_bytes = 0;
    baudrate_detection.detect_sentences = 0;
    baudrate_detection.detect_line_errors_start = rx.line_errors;
    baudrate_detection.state_start_tick = phytime_now();
}

static void baudrate_start_detection(void)
{
    gps.link = GPS_LINK_DETECTING;
    gps.baudrate = 0;
    baudrate_detection.candidate_index = 0;
    baudrate_start_candidate();
}

static void baudrate_try_next(void)
{
    ++baudrate_detection.candidate_index;
    baudrate_start_candidate();
}

static void detection_on_byte(const char *sentence)
{
    ++baudrate_detection.detect_bytes;
    if (sentence)
        ++baudrate_detection.detect_sentences;
}

static void baudrate_detection_check(void)
{
    // Need multiple sentences with a valid checksum to accept a valid rate.
    if (baudrate_detection.detect_sentences >= 2)
    {
        gps.link = GPS_LINK_RUNNING;
        gps.baudrate = candidate_baudrates[baudrate_detection.candidate_index];
        baudrate_detection.last_sentence_tick = phytime_now();
        baudrate_detection.errors_since_sentence = 0;
        log_info("GPS: detected %u baud" LOG_EOL, gps.baudrate);
        return;
    }

    // Give up on a rate early once it clearly produces garbage.
    const uint8_t max_line_errors = 8;
    const uint16_t max_bytes_without_sentence = 400;
    const bool garbage =
        (rx.line_errors - baudrate_detection.detect_line_errors_start) >= max_line_errors ||
        (baudrate_detection.detect_sentences == 0 && baudrate_detection.detect_bytes >= max_bytes_without_sentence);
    uint32_t time = ms_since(baudrate_detection.state_start_tick);
    if (garbage || time >= DETECT_WINDOW_MS)
    {
        baudrate_try_next();
    }
}

static void gps_data_set_valid(gps_data_aggregate_t *data, uint8_t section, bool valid)
{
    if (valid)
        data->valid |= section;
    else
        data->valid &= (uint8_t)~section;
}

static void gps_data_update(gps_data_aggregate_t *data, const nmea_message_t *msg)
{
    switch (msg->type)
    {
    case NMEA_MSG_RMC:
        data->time = msg->rmc.time;
        data->date = msg->rmc.date;
        data->position = msg->rmc.position;
        data->active = msg->rmc.active;
        data->speed_knots_milli = msg->rmc.speed_knots_milli;
        data->course_centideg = msg->rmc.course_centideg;
        gps_data_set_valid(data, GPS_DATA_VALID_TIME, msg->rmc.time_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_DATE, msg->rmc.date_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_POSITION, msg->rmc.position_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_SPEED, msg->rmc.speed_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_COURSE, msg->rmc.course_valid);
        break;
    case NMEA_MSG_GGA:
        data->time = msg->gga.time;
        data->position = msg->gga.position;
        data->altitude_mm = msg->gga.altitude_mm;
        data->geoid_separation_mm = msg->gga.geoid_separation_mm;
        data->fix_quality = msg->gga.fix_quality;
        data->satellites_used = msg->gga.satellites_used;
        gps_data_set_valid(data, GPS_DATA_VALID_TIME, msg->gga.time_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_POSITION, msg->gga.position_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_ALTITUDE, msg->gga.altitude_valid);
        break;
    case NMEA_MSG_GSA:
        // With several constellations the receiver emits one GSA per system, they share the DOP.
        data->fix_type = msg->gsa.fix_type;
        data->pdop_centi = msg->gsa.pdop_centi;
        data->hdop_centi = msg->gsa.hdop_centi;
        data->vdop_centi = msg->gsa.vdop_centi;
        gps_data_set_valid(data, GPS_DATA_VALID_DOP, msg->gsa.dop_valid);
        break;
    case NMEA_MSG_GSV:
        data->satellites_in_view[msg->system] = msg->gsv.satellites_in_view;
        break;
    case NMEA_MSG_ZDA:
        data->time = msg->zda.time;
        data->date = msg->zda.date;
        gps_data_set_valid(data, GPS_DATA_VALID_TIME, msg->zda.time_valid);
        gps_data_set_valid(data, GPS_DATA_VALID_DATE, msg->zda.date_valid);
        break;
    case NMEA_MSG_PMTK_SYS:
        log_info("GPS: system message %u" LOG_EOL, msg->pmtk_sys.message);
        break;
    default:
        break;
    }
}

static void message_buffer_put(const char *data, size_t length)
{
    for (size_t i = 0; i < length; ++i)
        messages.data[messages.head++ & (MESSAGE_BUFFER_SIZE - 1)] = data[i];
}

// Stores the sentence as received, "$<body>*HH\r\n". Whole sentences only, so the host never sees a partial one.
static void message_buffer_push(const char *body)
{
    static const char hex[] = "0123456789ABCDEF";
    const size_t length = strlen(body);
    if (MESSAGE_BUFFER_SIZE - (messages.head - messages.tail) < length + 6)
    {
        ++messages.dropped;
        return;
    }
    const uint8_t checksum = nmea_checksum(body, length);
    const char trailer[] = { '*', hex[checksum >> 4], hex[checksum & 0xF], '\r', '\n' };
    message_buffer_put("$", 1);
    message_buffer_put(body, length);
    message_buffer_put(trailer, sizeof(trailer));
}

static void running_on_byte(const char *sentence)
{
    if (!sentence)
        return;

    baudrate_detection.last_sentence_tick = phytime_now();
    baudrate_detection.errors_since_sentence = 0;
    message_buffer_push(sentence);

    nmea_message_t msg;
    if (!nmea_parse(sentence, &msg))
    {
        ++gps.stats.parse_errors;
        log_dbg("GPS: malformed sentence %s" LOG_EOL, sentence);
        return;
    }

    const bool had_fix = gps.data.active;
    gps_data_update(&gps.data, &msg);
    if (gps.data.active != had_fix)
        log_info("GPS: fix %s" LOG_EOL, gps.data.active ? "acquired" : "lost");

    // TODO: do something if fix is lost?
}

static void gps_link_check(uint32_t new_errors)
{
    baudrate_detection.errors_since_sentence += new_errors;
    if (baudrate_detection.errors_since_sentence >= LINK_LOST_MIN_ERRORS &&
        ms_since(baudrate_detection.last_sentence_tick) >= LINK_LOST_MS)
    {
        // Silence alone is not a lost link, the module may be in standby.
        log_err("GPS: receiving only invalid data at %u baud, redetecting" LOG_EOL, gps.baudrate);
        baudrate_start_detection();
    }
}

void gps_initialize(uint32_t uart_clock_hz)
{
    memset(&gps, 0, sizeof(gps));
    uart_clock = uart_clock_hz;
    NVIC_SetPriority(IRQ_UART, GPS_UART_IRQ_PRIORITY);
    // Detection needs the scheduler for timing, it starts on the first gps_process() call.
    gps.link = GPS_LINK_DETECTING;
    baudrate_detection.start_pending = true;
}

void gps_process(void)
{
    if (gps.link == GPS_LINK_IDLE || gps.link == GPS_LINK_FAIL)
        return;

    if (baudrate_detection.start_pending)
    {
        baudrate_detection.start_pending = false;
        baudrate_start_detection();
        return;
    }

    const uint32_t line_errors_before = gps.stats.rx_line_errors;
    gps.stats.rx_line_errors = rx.line_errors;
    gps.stats.rx_buffer_overflows = rx.overflows;

    const uint32_t head = rx.head;
    uint32_t tail = rx.tail;
    while (tail != head)
    {
        const uint8_t byte = rx.data[tail & (RX_BUFFER_SIZE - 1)];
        ++tail;
        ++gps.stats.rx_bytes;
        const char *sentence = nmea_framer_feed(&framer, byte);
        if (gps.link == GPS_LINK_DETECTING)
            detection_on_byte(sentence);
        else
            running_on_byte(sentence);
    }
    rx.tail = tail;

    const uint32_t framer_errors = framer.checksum_errors + framer.framing_errors;
    gps.stats.sentences_ok += framer.sentences_ok;
    gps.stats.checksum_errors += framer.checksum_errors;
    gps.stats.framing_errors += framer.framing_errors;
    framer.sentences_ok = 0;
    framer.checksum_errors = 0;
    framer.framing_errors = 0;

    if (gps.link == GPS_LINK_DETECTING)
        baudrate_detection_check();
    else
        gps_link_check(framer_errors + (gps.stats.rx_line_errors - line_errors_before));
}

void gps_set_uart_clock(uint32_t uart_clock_hz)
{
    uart_clock = uart_clock_hz;
    switch (gps.link)
    {
    case GPS_LINK_RUNNING:
        if (!is_baudrate_achievable(gps.baudrate))
        {
            log_err("GPS: %u baud not possible with %u Hz UART clock" LOG_EOL, gps.baudrate, uart_clock_hz);
            baudrate_start_detection();
            break;
        }
        uart_set_baudrate(gps.baudrate);
        baudrate_detection.last_sentence_tick = phytime_now();
        break;
    case GPS_LINK_DETECTING:
        if (!baudrate_detection.start_pending)
            baudrate_start_candidate(); // restart the current candidate's window
        break;
    default:
        break;
    }
}

bool gps_send_sentence(const char *body)
{
    const size_t length = strlen(body);
    if (gps.link != GPS_LINK_RUNNING || length > NMEA_MAX_SENTENCE_LEN - 3)
        return false;
    uart_write_sentence(body, length);
    return true;
}

const gps_state_t *gps_get_state(void)
{
    return &gps;
}
