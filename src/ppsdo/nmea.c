#include "nmea.h"

#include <string.h>

typedef struct nmea_field {
    const char *str;
    uint8_t length;
} nmea_field_t;

void nmea_framer_reset(nmea_framer_t *framer)
{
    memset(framer, 0, sizeof(*framer));
}

uint8_t nmea_checksum(const char *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i)
        checksum ^= (uint8_t)data[i];
    return checksum;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

// Validates the "<body>*HH" in the buffer and terminates it at '*'.
static const char *framer_complete(nmea_framer_t *framer)
{
    const uint16_t length = framer->length;
    if (length < 4 || framer->buffer[length - 3] != '*')
    {
        ++framer->framing_errors;
        return NULL;
    }

    const int hi = hex_value(framer->buffer[length - 2]);
    const int lo = hex_value(framer->buffer[length - 1]);
    const uint16_t body_length = length - 3;
    if (hi < 0 || lo < 0 || nmea_checksum(framer->buffer, body_length) != ((hi << 4) | lo))
    {
        ++framer->checksum_errors;
        return NULL;
    }

    framer->buffer[body_length] = '\0';
    ++framer->sentences_ok;
    return framer->buffer;
}

const char *nmea_framer_feed(nmea_framer_t *framer, uint8_t byte)
{
    if (byte == '$')
    {
        if (framer->in_sentence)
            ++framer->framing_errors; // previous sentence was not terminated
        framer->in_sentence = true;
        framer->length = 0;
        return NULL;
    }

    if (!framer->in_sentence)
        return NULL;

    if (byte == '\r' || byte == '\n')
    {
        framer->in_sentence = false;
        return framer_complete(framer);
    }

    // Anything outside printable ASCII means line noise or a wrong baud rate.
    if (byte < 0x20 || byte > 0x7E || framer->length >= NMEA_MAX_SENTENCE_LEN)
    {
        framer->in_sentence = false;
        ++framer->framing_errors;
        return NULL;
    }

    framer->buffer[framer->length++] = (char)byte;
    return NULL;
}

static int split_fields(const char *sentence, nmea_field_t *fields, int max_fields)
{
    int count = 0;
    const char *start = sentence;
    for (const char *p = sentence;; ++p)
    {
        if (*p != ',' && *p != '\0')
            continue;
        if (count == max_fields)
            break;
        fields[count].str = start;
        fields[count].length = (uint8_t)(p - start);
        ++count;
        if (*p == '\0')
            break;
        start = p + 1;
    }
    return count;
}

static bool parse_uint(const nmea_field_t *field, uint32_t *value)
{
    if (field->length == 0)
        return false;
    uint32_t result = 0;
    for (uint8_t i = 0; i < field->length; ++i)
    {
        const char c = field->str[i];
        if (c < '0' || c > '9')
            return false;
        result = result * 10 + (uint32_t)(c - '0');
    }
    *value = result;
    return true;
}

// Parses a decimal number "[-]123.456" into an integer scaled by 10^decimals.
// Extra fractional digits are truncated.
static bool parse_fixed(const nmea_field_t *field, uint8_t decimals, int64_t *value)
{
    if (field->length == 0)
        return false;

    uint8_t i = 0;
    bool negative = false;
    if (field->str[0] == '-' || field->str[0] == '+')
    {
        negative = field->str[0] == '-';
        ++i;
    }

    int64_t result = 0;
    bool seen_point = false;
    bool seen_digit = false;
    uint8_t fraction_digits = 0;
    for (; i < field->length; ++i)
    {
        const char c = field->str[i];
        if (c == '.' && !seen_point)
        {
            seen_point = true;
            continue;
        }
        if (c < '0' || c > '9')
            return false;
        seen_digit = true;
        if (seen_point)
        {
            if (fraction_digits == decimals)
                continue;
            ++fraction_digits;
        }
        result = result * 10 + (c - '0');
    }
    if (!seen_digit)
        return false;

    for (; fraction_digits < decimals; ++fraction_digits)
        result *= 10;
    *value = negative ? -result : result;
    return true;
}

static uint32_t fixed_u32_or_zero(const nmea_field_t *field, uint8_t decimals)
{
    int64_t value;
    if (!parse_fixed(field, decimals, &value) || value < 0 || value > UINT32_MAX)
        return 0;
    return (uint32_t)value;
}

static uint16_t fixed_u16_or_zero(const nmea_field_t *field, uint8_t decimals)
{
    const uint32_t value = fixed_u32_or_zero(field, decimals);
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static uint32_t uint_or_zero(const nmea_field_t *field)
{
    uint32_t value;
    return parse_uint(field, &value) ? value : 0;
}

static char char_or_zero(const nmea_field_t *field)
{
    return field->length > 0 ? field->str[0] : 0;
}

static uint8_t digit_pair(const char *s)
{
    return (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
}

static bool all_digits(const char *s, uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
        if (s[i] < '0' || s[i] > '9')
            return false;
    return true;
}

// "hhmmss" or "hhmmss.sss"
static bool parse_time(const nmea_field_t *field, nmea_time_t *time)
{
    memset(time, 0, sizeof(*time));
    if (field->length < 6 || !all_digits(field->str, 6))
        return field->length == 0;

    time->hour = digit_pair(field->str);
    time->minute = digit_pair(field->str + 2);
    time->second = digit_pair(field->str + 4);
    if (time->hour > 23 || time->minute > 59 || time->second > 60)
        return false;

    if (field->length > 6)
    {
        if (field->str[6] != '.')
            return false;
        const nmea_field_t fraction = { .str = field->str + 6, .length = (uint8_t)(field->length - 6) };
        int64_t ms;
        if (!parse_fixed(&fraction, 3, &ms))
            return false;
        time->millisecond = (uint16_t)ms;
    }
    return true;
}

// "ddmmyy"
static bool parse_date(const nmea_field_t *field, nmea_date_t *date)
{
    memset(date, 0, sizeof(*date));
    if (field->length == 0)
        return true;
    if (field->length != 6 || !all_digits(field->str, 6))
        return false;

    date->day = digit_pair(field->str);
    date->month = digit_pair(field->str + 2);
    const uint8_t yy = digit_pair(field->str + 4);
    // Receivers report dates near the GPS epoch (1980) before they have synced time.
    date->year = yy >= 80 ? 1900 + yy : 2000 + yy;
    if (date->day < 1 || date->day > 31 || date->month < 1 || date->month > 12)
        return false;
    return true;
}

// "dddmm.mmmm" + hemisphere into degrees * 1e7
static bool parse_coordinate(const nmea_field_t *value, const nmea_field_t *hemisphere, char negative_hemisphere, int32_t *result)
{
    int64_t ddmm; // ddmm.mmmmmmm * 1e7
    if (!parse_fixed(value, 7, &ddmm) || ddmm < 0 || hemisphere->length != 1)
        return false;

    const int64_t degrees = ddmm / 1000000000LL;
    const int64_t minutes_e7 = ddmm - degrees * 1000000000LL;
    if (minutes_e7 >= 600000000LL)
        return false;

    int64_t scaled = degrees * 10000000LL + (minutes_e7 + 30) / 60;
    if (hemisphere->str[0] == negative_hemisphere)
        scaled = -scaled;
    *result = (int32_t)scaled;
    return true;
}

static bool parse_position(const nmea_field_t *fields, nmea_position_t *position)
{
    memset(position, 0, sizeof(*position));
    if (fields[0].length == 0 && fields[2].length == 0)
        return true; // no fix
    if (!parse_coordinate(&fields[0], &fields[1], 'S', &position->latitude_1e7) ||
        !parse_coordinate(&fields[2], &fields[3], 'W', &position->longitude_1e7))
        return false;
    if (position->latitude_1e7 > 900000000 || position->latitude_1e7 < -900000000 || position->longitude_1e7 > 1800000000 ||
        position->longitude_1e7 < -1800000000)
        return false;
    return true;
}

static bool parse_rmc(const nmea_field_t *f, int n, nmea_rmc_t *rmc)
{
    if (n < 10)
        return false;
    if (!parse_time(&f[1], &rmc->time) || !parse_position(&f[3], &rmc->position) || !parse_date(&f[9], &rmc->date))
        return false;
    rmc->time_valid = f[1].length > 0;
    rmc->position_valid = f[3].length > 0;
    rmc->date_valid = f[9].length > 0;
    rmc->active = char_or_zero(&f[2]) == 'A';
    rmc->speed_knots_milli = fixed_u32_or_zero(&f[7], 3);
    rmc->speed_valid = f[7].length > 0;
    rmc->course_centideg = fixed_u32_or_zero(&f[8], 2);
    rmc->course_valid = f[8].length > 0;
    if (n > 11)
    {
        int64_t variation;
        if (parse_fixed(&f[10], 2, &variation))
            rmc->magnetic_variation_centideg = (int32_t)(char_or_zero(&f[11]) == 'W' ? -variation : variation);
    }
    if (n > 12)
        rmc->mode = char_or_zero(&f[12]);
    return true;
}

static bool parse_gga(const nmea_field_t *f, int n, nmea_gga_t *gga)
{
    if (n < 12)
        return false;
    if (!parse_time(&f[1], &gga->time) || !parse_position(&f[2], &gga->position))
        return false;
    gga->time_valid = f[1].length > 0;
    gga->position_valid = f[2].length > 0;
    gga->fix_quality = (uint8_t)uint_or_zero(&f[6]);
    gga->satellites_used = (uint8_t)uint_or_zero(&f[7]);
    gga->hdop_centi = fixed_u16_or_zero(&f[8], 2);

    int64_t value;
    gga->altitude_valid = parse_fixed(&f[9], 3, &value);
    if (gga->altitude_valid)
        gga->altitude_mm = (int32_t)value;
    else
        gga->altitude_mm = 0;

    if (parse_fixed(&f[11], 3, &value))
        gga->geoid_separation_mm = (int32_t)value;
    else
        gga->geoid_separation_mm = 0;
    return true;
}

static bool parse_gsa(const nmea_field_t *f, int n, nmea_gsa_t *gsa)
{
    if (n < 18)
        return false;
    gsa->selection_mode = char_or_zero(&f[1]);
    gsa->fix_type = (uint8_t)uint_or_zero(&f[2]);
    for (int i = 0; i < NMEA_GSA_MAX_PRNS; ++i)
    {
        uint32_t prn;
        if (parse_uint(&f[3 + i], &prn))
            gsa->prn[gsa->prn_count++] = (uint16_t)prn;
    }
    gsa->pdop_centi = fixed_u16_or_zero(&f[15], 2);
    gsa->hdop_centi = fixed_u16_or_zero(&f[16], 2);
    gsa->vdop_centi = fixed_u16_or_zero(&f[17], 2);
    gsa->dop_valid = f[15].length > 0 && f[16].length > 0 && f[17].length > 0;
    if (n > 18)
        gsa->system_id = (uint8_t)uint_or_zero(&f[18]);
    return true;
}

static bool parse_gsv(const nmea_field_t *f, int n, nmea_gsv_t *gsv)
{
    if (n < 4)
        return false;
    uint32_t total, number, in_view;
    if (!parse_uint(&f[1], &total) || !parse_uint(&f[2], &number) || !parse_uint(&f[3], &in_view))
        return false;
    gsv->total_messages = (uint8_t)total;
    gsv->message_number = (uint8_t)number;
    gsv->satellites_in_view = (uint8_t)in_view;

    // Groups of 4 fields; NMEA 4.1 adds a trailing signal ID field, which is ignored.
    for (int i = 4; i + 4 <= n && gsv->satellite_count < NMEA_GSV_SATS_PER_MSG; i += 4)
    {
        uint32_t prn, value;
        if (!parse_uint(&f[i], &prn))
            continue;
        nmea_satellite_t *sat = &gsv->satellites[gsv->satellite_count++];
        sat->prn = (uint16_t)prn;
        sat->elevation_deg = parse_uint(&f[i + 1], &value) ? (int8_t)value : -1;
        sat->azimuth_deg = parse_uint(&f[i + 2], &value) ? (int16_t)value : -1;
        sat->snr_db = parse_uint(&f[i + 3], &value) ? (int8_t)value : -1;
    }
    return true;
}

static bool parse_vtg(const nmea_field_t *f, int n, nmea_vtg_t *vtg)
{
    if (n < 9)
        return false;
    vtg->course_true_centideg = fixed_u32_or_zero(&f[1], 2);
    vtg->course_magnetic_centideg = fixed_u32_or_zero(&f[3], 2);
    vtg->speed_knots_milli = fixed_u32_or_zero(&f[5], 3);
    vtg->speed_kmh_milli = fixed_u32_or_zero(&f[7], 3);
    if (n > 9)
        vtg->mode = char_or_zero(&f[9]);
    return true;
}

static bool parse_gll(const nmea_field_t *f, int n, nmea_gll_t *gll)
{
    if (n < 7)
        return false;
    if (!parse_position(&f[1], &gll->position) || !parse_time(&f[5], &gll->time))
        return false;
    gll->active = char_or_zero(&f[6]) == 'A';
    if (n > 7)
        gll->mode = char_or_zero(&f[7]);
    return true;
}

static bool parse_zda(const nmea_field_t *f, int n, nmea_zda_t *zda)
{
    if (n < 7)
        return false;
    if (!parse_time(&f[1], &zda->time))
        return false;
    zda->time_valid = f[1].length > 0;
    uint32_t day, month, year;
    if (parse_uint(&f[2], &day) && parse_uint(&f[3], &month) && parse_uint(&f[4], &year) && day >= 1 && day <= 31 && month >= 1 &&
        month <= 12)
    {
        zda->date.day = (uint8_t)day;
        zda->date.month = (uint8_t)month;
        zda->date.year = (uint16_t)year;
        zda->date_valid = true;
    }
    int64_t zone_hours;
    if (parse_fixed(&f[5], 0, &zone_hours))
        zda->local_zone_hours = (int8_t)zone_hours;
    zda->local_zone_minutes = (uint8_t)uint_or_zero(&f[6]);
    return true;
}

static nmea_system_t talker_to_system(const char *talker)
{
    static const struct {
        char id[3];
        nmea_system_t system;
    } talkers[] = {
        { "GP", NMEA_SYSTEM_GPS },
        { "GL", NMEA_SYSTEM_GLONASS },
        { "GA", NMEA_SYSTEM_GALILEO },
        { "GB", NMEA_SYSTEM_BEIDOU },
        { "BD", NMEA_SYSTEM_BEIDOU },
        { "GQ", NMEA_SYSTEM_QZSS },
        { "GN", NMEA_SYSTEM_COMBINED },
    };
    for (size_t i = 0; i < sizeof(talkers) / sizeof(talkers[0]); ++i)
        if (talker[0] == talkers[i].id[0] && talker[1] == talkers[i].id[1])
            return talkers[i].system;
    return NMEA_SYSTEM_UNKNOWN;
}

static bool parse_pmtk(const nmea_field_t *f, int n, nmea_message_t *msg)
{
    uint32_t value;
    if (f[0].length != 7 || !parse_uint(&(nmea_field_t){ .str = f[0].str + 4, .length = 3 }, &value))
        return true; // unknown

    if (value == 1)
    {
        uint32_t command, flag;
        if (n < 3 || !parse_uint(&f[1], &command) || !parse_uint(&f[2], &flag))
            return false;
        msg->type = NMEA_MSG_PMTK_ACK;
        msg->pmtk_ack.command = (uint16_t)command;
        msg->pmtk_ack.flag = (uint8_t)flag;
    }
    else if (value == 10)
    {
        uint32_t message;
        if (n < 2 || !parse_uint(&f[1], &message))
            return false;
        msg->type = NMEA_MSG_PMTK_SYS;
        msg->pmtk_sys.message = (uint8_t)message;
    }
    return true;
}

bool nmea_parse(const char *sentence, nmea_message_t *msg)
{
    memset(msg, 0, sizeof(*msg));

    nmea_field_t fields[NMEA_MAX_FIELDS];
    const int n = split_fields(sentence, fields, NMEA_MAX_FIELDS);
    const nmea_field_t *address = &fields[0];

    if (address->length >= 4 && strncmp(address->str, "PMTK", 4) == 0)
        return parse_pmtk(fields, n, msg);

    if (address->length != 5)
        return true; // other proprietary or unknown sentence

    msg->system = talker_to_system(address->str);
    const char *type = address->str + 2;
    static const struct {
        char id[4];
        nmea_msg_type_t type;
    } types[] = {
        { "RMC", NMEA_MSG_RMC },
        { "GGA", NMEA_MSG_GGA },
        { "GSA", NMEA_MSG_GSA },
        { "GSV", NMEA_MSG_GSV },
        { "VTG", NMEA_MSG_VTG },
        { "GLL", NMEA_MSG_GLL },
        { "ZDA", NMEA_MSG_ZDA },
    };
    for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); ++i)
    {
        if (strncmp(type, types[i].id, 3) == 0)
        {
            msg->type = types[i].type;
            break;
        }
    }

    bool ok = true;
    switch (msg->type)
    {
    case NMEA_MSG_RMC:
        ok = parse_rmc(fields, n, &msg->rmc);
        break;
    case NMEA_MSG_GGA:
        ok = parse_gga(fields, n, &msg->gga);
        break;
    case NMEA_MSG_GSA:
        ok = parse_gsa(fields, n, &msg->gsa);
        break;
    case NMEA_MSG_GSV:
        ok = parse_gsv(fields, n, &msg->gsv);
        break;
    case NMEA_MSG_VTG:
        ok = parse_vtg(fields, n, &msg->vtg);
        break;
    case NMEA_MSG_GLL:
        ok = parse_gll(fields, n, &msg->gll);
        break;
    case NMEA_MSG_ZDA:
        ok = parse_zda(fields, n, &msg->zda);
        break;
    default:
        break;
    }
    if (!ok)
        msg->type = NMEA_MSG_UNKNOWN;
    return ok;
}

const char *nmea_system_name(nmea_system_t system)
{
    switch (system)
    {
    case NMEA_SYSTEM_GPS:
        return "GPS";
    case NMEA_SYSTEM_GLONASS:
        return "GLONASS";
    case NMEA_SYSTEM_GALILEO:
        return "Galileo";
    case NMEA_SYSTEM_BEIDOU:
        return "BeiDou";
    case NMEA_SYSTEM_QZSS:
        return "QZSS";
    case NMEA_SYSTEM_COMBINED:
        return "GNSS";
    default:
        return "unknown";
    }
}
