/*
 * AUX HVAC UART protocol (Wi‑Fi module serial protocol)
 * Based on: https://github.com/GrKoR/esphome_aux_ac_component / AUX_HVAC_Protocol
 */

#include "hlink_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "AUX_DRV";

/* Set to 1 in hlink_driver.c or via compiler -D to log every TX/RX frame at INFO (very noisy on USB). */
#ifndef AUX_LOG_ALL_UART_FRAMES
#define AUX_LOG_ALL_UART_FRAMES 0
#endif

/* ---- Protocol constants (aux_ac.h) ---- */
#define AC_HEADER_SIZE          8
#define AC_BUFFER_SIZE          35
#define AC_PACKET_START_BYTE    0xBB
#define AC_PACKET_ANSWER        0x80

#define AC_PTYPE_PING           0x01
#define AC_PTYPE_CMD            0x06
#define AC_PTYPE_INFO           0x07

#define AC_CMD_SET_PARAMS       0x01
#define AC_CMD_STATUS_SMALL     0x11
#define AC_CMD_STATUS_BIG       0x21
#define AC_CMD_STATUS_PERIODIC  0x2C

#define AC_POWER_MASK           0x20U
#define AC_POWER_ON             0x20U
#define AC_POWER_OFF            0x00U
#define AC_MODE_MASK            0xE0U
#define AC_MODE_AUTO            0x00U
#define AC_MODE_COOL            0x20U
#define AC_MODE_DRY              0x40U
#define AC_MODE_HEAT            0x80U
#define AC_MODE_FAN             0xC0U
#define AC_FANSPEED_MASK        0xE0U
#define AC_FANSPEED_HIGH        0x20U
#define AC_FANSPEED_MEDIUM      0x40U
#define AC_FANSPEED_LOW         0x60U
#define AC_FANSPEED_AUTO        0xA0U
#define AC_FANTURBO_MASK        0x40U
#define AC_FANMUTE_MASK         0x80U
#define AC_SLEEP_MASK           0x04U
#define AC_TEMPERATURE_UNIT_MASK 0x02U
#define AC_TEMP_TARGET_INT_PART_MASK 0xF8U
#define AC_TEMP_TARGET_FRAC_PART_MASK 0x80U
#define AC_LOUVERV_MASK         0x07U
#define AC_LOUVERH_MASK         0xE0U
#define AC_LOUVERV_SWING_UPDOWN 0x00U
#define AC_LOUVERV_OFF          0x07U
#define AC_LOUVERH_SWING_LEFTRIGHT 0x00U
#define AC_LOUVERH_OFF_ALT      0xE0U
#define AC_POWLIMSTAT_MASK      0x80U
#define AC_POWLIMVAL_MASK       0x7FU

#define AC_REAL_FAN_OFF         0x00U
#define AC_REAL_FAN_MUTE        0x01U

typedef enum {
    ACSM_IDLE = 0,
    ACSM_RECEIVING_PACKET,
    ACSM_PARSING_PACKET,
    ACSM_SENDING_PACKET,
} acsm_state_t;

typedef struct {
    uint8_t data[AC_BUFFER_SIZE];
    uint32_t msec;
    uint8_t bytes_loaded;
} aux_packet_t;

static hlink_state_t s_zb;
static SemaphoreHandle_t s_state_mutex;
static void (*s_state_cb)(void) = NULL;

static TaskHandle_t s_uart_task;
static acsm_state_t s_acsm;
static aux_packet_t s_in;
static aux_packet_t s_out;
static uint32_t s_packet_timeout_ms = HLINK_PACKET_TIMEOUT_MS;
static bool s_initialized;
static bool s_have_status;
/* If false, we only respond to inbound PING and never poll — AC would stay uncontrollable.
 * ESPHome aux_ac polls proactively; indoor unit may never send PING first. */
static bool s_has_connection = true;
static bool s_is_inverter;
static uint32_t s_last_poll_ms;

/* Last decoded AUX state for building SET commands */
static float s_temp_ambient;
static float s_temp_inbound;
static int8_t s_temp_outdoor;
static uint8_t s_real_fan;
static uint8_t s_inverter_power;

/* Last small-status body (15 bytes) — template for SET 0x01 */
static uint8_t s_raw_body[16];
/* Don't let status poll overwrite fan_mode right after a Zigbee-driven change */
static uint32_t s_fan_user_override_until_ms;

static inline uint32_t millis(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* CRC16: sum of 16-bit words, fold, invert (same as esphome_aux_ac) */
static uint16_t aux_crc16(const uint8_t *data, uint8_t len)
{
    uint32_t crc = 0;
    uint8_t buf[AC_BUFFER_SIZE];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, data, len);
    if ((len % 2) == 1) {
        len++;
    }
    for (uint8_t i = 0; i < len; i += 2) {
        uint32_t word = ((uint32_t)buf[i] << 8) + buf[i + 1];
        crc += word;
    }
    crc = (crc >> 16) + (crc & 0xFFFF);
    crc = ~crc;
    return (uint16_t)(crc & 0xFFFF);
}

static void crc_set(aux_packet_t *p)
{
    uint8_t bl = p->data[6];
    uint16_t c = aux_crc16(p->data, (uint8_t)(AC_HEADER_SIZE + bl));
    uint8_t *crcp = &p->data[AC_HEADER_SIZE + bl];
    crcp[0] = (uint8_t)((c >> 8) & 0xFF);
    crcp[1] = (uint8_t)(c & 0xFF);
}

static bool crc_ok(const aux_packet_t *p)
{
    uint8_t bl = p->data[6];
    if (p->bytes_loaded < AC_HEADER_SIZE + bl + 2) {
        return false;
    }
    uint16_t c = aux_crc16(p->data, (uint8_t)(AC_HEADER_SIZE + bl));
    const uint8_t *crcp = &p->data[AC_HEADER_SIZE + bl];
    /* On wire: [crc_hi, crc_lo] same as esphome _checkCRC */
    return (crcp[0] == (uint8_t)((c >> 8) & 0xFF)) && (crcp[1] == (uint8_t)(c & 0xFF));
}

static void pkt_clear(aux_packet_t *p)
{
    memset(p->data, 0, sizeof(p->data));
    p->bytes_loaded = 0;
    p->msec = 0;
}

static void out_prepare_cmd_header(aux_packet_t *p, uint8_t body_len)
{
    p->data[0] = AC_PACKET_START_BYTE;
    p->data[1] = 0;
    p->data[2] = AC_PTYPE_CMD;
    p->data[3] = AC_PACKET_ANSWER;
    p->data[4] = 0;
    p->data[5] = 0;
    p->data[6] = body_len;
    p->data[7] = 0;
}

static void fill_status_small(aux_packet_t *p)
{
    pkt_clear(p);
    out_prepare_cmd_header(p, 2);
    p->data[AC_HEADER_SIZE + 0] = AC_CMD_STATUS_SMALL;
    p->data[AC_HEADER_SIZE + 1] = 0x01;
    p->bytes_loaded = AC_HEADER_SIZE + 2 + 2;
    crc_set(p);
    p->msec = millis();
}

static void fill_status_big(aux_packet_t *p)
{
    pkt_clear(p);
    out_prepare_cmd_header(p, 2);
    p->data[AC_HEADER_SIZE + 0] = AC_CMD_STATUS_BIG;
    p->data[AC_HEADER_SIZE + 1] = 0x01;
    p->bytes_loaded = AC_HEADER_SIZE + 2 + 2;
    crc_set(p);
    p->msec = millis();
}

static float clamp_temp(float t)
{
    if (t < 16.0f) {
        return 16.0f;
    }
    if (t > 32.0f) {
        return 32.0f;
    }
    return t;
}

static uint8_t zb_mode_to_ac(hlink_zb_mode_t m)
{
    switch (m) {
        case HLINK_ZB_MODE_COOL: return AC_MODE_COOL;
        case HLINK_ZB_MODE_HEAT: return AC_MODE_HEAT;
        case HLINK_ZB_MODE_DRY: return AC_MODE_DRY;
        case HLINK_ZB_MODE_FAN_ONLY: return AC_MODE_FAN;
        case HLINK_ZB_MODE_AUTO:
        default: return AC_MODE_AUTO;
    }
}

static hlink_zb_mode_t ac_mode_to_zb(uint8_t acm)
{
    switch (acm & AC_MODE_MASK) {
        case AC_MODE_COOL: return HLINK_ZB_MODE_COOL;
        case AC_MODE_HEAT: return HLINK_ZB_MODE_HEAT;
        case AC_MODE_DRY: return HLINK_ZB_MODE_DRY;
        case AC_MODE_FAN: return HLINK_ZB_MODE_FAN_ONLY;
        case AC_MODE_AUTO:
        default: return HLINK_ZB_MODE_AUTO;
    }
}

static uint8_t zb_fan_to_ac(hlink_zb_fan_t f)
{
    switch (f) {
        case HLINK_ZB_FAN_HIGH: return AC_FANSPEED_HIGH;
        case HLINK_ZB_FAN_MEDIUM: return AC_FANSPEED_MEDIUM;
        case HLINK_ZB_FAN_LOW: return AC_FANSPEED_LOW;
        case HLINK_ZB_FAN_QUIET: return AC_FANSPEED_LOW;
        case HLINK_ZB_FAN_AUTO:
        default: return AC_FANSPEED_AUTO;
    }
}

static hlink_zb_fan_t ac_fan_to_zb(uint8_t fs)
{
    switch (fs & AC_FANSPEED_MASK) {
        case AC_FANSPEED_HIGH: return HLINK_ZB_FAN_HIGH;
        case AC_FANSPEED_MEDIUM: return HLINK_ZB_FAN_MEDIUM;
        case AC_FANSPEED_LOW: return HLINK_ZB_FAN_LOW;
        case AC_FANSPEED_AUTO: return HLINK_ZB_FAN_AUTO;
        default: return HLINK_ZB_FAN_AUTO;
    }
}

static void swing_to_louvers(uint8_t swing, uint8_t *lv, uint8_t *lh)
{
    switch (swing) {
        case 1:
            *lv = AC_LOUVERV_SWING_UPDOWN;
            *lh = AC_LOUVERH_OFF_ALT;
            break;
        case 2:
            *lv = AC_LOUVERV_OFF;
            *lh = AC_LOUVERH_SWING_LEFTRIGHT;
            break;
        case 3:
            *lv = AC_LOUVERV_SWING_UPDOWN;
            *lh = AC_LOUVERH_SWING_LEFTRIGHT;
            break;
        case 0:
        default:
            *lv = AC_LOUVERV_OFF;
            *lh = AC_LOUVERH_OFF_ALT;
            break;
    }
}

static void update_running_state(void)
{
    if (!s_zb.power_on) {
        s_zb.running_state = HLINK_RUNNING_IDLE;
        return;
    }
    int16_t delta = (int16_t)(s_temp_ambient - s_temp_inbound);
    if (s_zb.mode == HLINK_ZB_MODE_DRY) {
        s_zb.running_state = HLINK_RUNNING_DRYING;
        return;
    }
    if (s_zb.mode == HLINK_ZB_MODE_FAN_ONLY) {
        s_zb.running_state = HLINK_RUNNING_FAN_ONLY;
        return;
    }
    if (s_real_fan != AC_REAL_FAN_OFF && s_real_fan != AC_REAL_FAN_MUTE) {
        if (delta > 2) {
            s_zb.running_state = HLINK_RUNNING_COOLING;
        } else if (delta < -2) {
            s_zb.running_state = HLINK_RUNNING_HEATING;
        } else {
            s_zb.running_state = HLINK_RUNNING_FAN_ONLY;
        }
    } else {
        s_zb.running_state = HLINK_RUNNING_IDLE;
    }
}

/* Parse 0x11 small INFO body (15 bytes) — offsets match packet_small_info_body_t */
static void parse_small_body(const uint8_t *b, size_t len)
{
    if (len < 15) {
        return;
    }
    memcpy(s_raw_body, b, 15);

    uint8_t t_int = (uint8_t)((b[2] >> 3) & 0x1F);
    float t_frac = (float)(b[14] % 10) / 10.0f;
    s_zb.target_temperature = 8.0f + (float)t_int + t_frac;

    s_zb.power_on = (b[10] & AC_POWER_MASK) != 0;
    s_zb.mode = ac_mode_to_zb(b[7]);
    s_zb.raw_ac_mode = (uint16_t)(b[7] & AC_MODE_MASK);
    {
        hlink_zb_fan_t parsed_fan = ac_fan_to_zb(b[5]);
        uint32_t now = millis();
        if (now < s_fan_user_override_until_ms) {
            if (parsed_fan == s_zb.fan_mode) {
                s_fan_user_override_until_ms = 0;
            }
        } else {
            s_zb.fan_mode = parsed_fan;
        }
    }

    uint8_t swing;
    uint8_t lv = b[2] & AC_LOUVERV_MASK;
    uint8_t lh = b[3] & AC_LOUVERH_MASK;
    if (lv == AC_LOUVERV_SWING_UPDOWN && (lh == AC_LOUVERH_SWING_LEFTRIGHT)) {
        swing = 3;
    } else if (lv == AC_LOUVERV_SWING_UPDOWN) {
        swing = 1;
    } else if (lh == AC_LOUVERH_SWING_LEFTRIGHT) {
        swing = 2;
    } else {
        swing = 0;
    }
    s_zb.swing_mode = swing;
    s_have_status = true;
    update_running_state();
}

/* Parse 0x21 big INFO body */
static void parse_big_body(const uint8_t *b, size_t len)
{
    if (len < 24) {
        return;
    }
    s_is_inverter = (b[2] >> 5) & 1;
    s_temp_ambient = (float)b[7] - 32.0f + (float)(b[23] & 0x0f) / 10.0f;
    s_temp_outdoor = (int8_t)((int)b[12] - 32);
    s_temp_inbound = (float)b[9] - 32.0f;
    s_zb.current_temperature = s_temp_ambient;
    s_zb.outdoor_temperature = (float)s_temp_outdoor;
    s_real_fan = b[5] & 0x07U;
    s_inverter_power = b[16];
    (void)s_is_inverter;
    (void)s_inverter_power;
    update_running_state();
}

static void fill_set_params(aux_packet_t *p)
{
    uint8_t body[16];
    if (!s_have_status) {
        return;
    }
    memcpy(body, s_raw_body, 15);
    body[0] = AC_CMD_SET_PARAMS;
    body[1] = 0x01;

    float tt = clamp_temp(s_zb.target_temperature);
    uint8_t ti = (uint8_t)tt;
    uint8_t lv, lh;
    swing_to_louvers(s_zb.swing_mode, &lv, &lh);
    uint8_t b2 = (body[2] & (uint8_t)~AC_TEMP_TARGET_INT_PART_MASK)
        | (uint8_t)(((ti - 8) & 0x1F) << 3);
    b2 = (b2 & (uint8_t)~AC_LOUVERV_MASK) | (lv & AC_LOUVERV_MASK);
    body[2] = b2;
    body[3] = (body[3] & (uint8_t)~AC_LOUVERH_MASK) | (lh & AC_LOUVERH_MASK);

    /* Byte 5 = preset fan; byte 6 = turbo/mute (GrKoR / AUX_HVAC_Protocol). If turbo stays set from
     * status, many indoor units ignore Low/Med/High on byte 5. */
    uint8_t ac_fan = zb_fan_to_ac(s_zb.fan_mode);
    body[5] = (body[5] & (uint8_t)~AC_FANSPEED_MASK) | (ac_fan & AC_FANSPEED_MASK);
    if (ac_fan == AC_FANSPEED_AUTO) {
        body[6] = (uint8_t)(body[6] & (uint8_t)~AC_FANTURBO_MASK);
    } else {
        body[6] = (uint8_t)(body[6] & (uint8_t)~(AC_FANTURBO_MASK | AC_FANMUTE_MASK));
    }

    body[7] = (body[7] & (uint8_t)~AC_MODE_MASK) | (zb_mode_to_ac(s_zb.mode) & AC_MODE_MASK);

    body[10] = (body[10] & (uint8_t)~AC_POWER_MASK)
        | (s_zb.power_on ? AC_POWER_ON : AC_POWER_OFF);

    if (tt - (float)ti >= 0.5f) {
        body[4] |= AC_TEMP_TARGET_FRAC_PART_MASK;
    } else {
        body[4] &= (uint8_t)~AC_TEMP_TARGET_FRAC_PART_MASK;
    }
    body[14] = (uint8_t)((uint8_t)(tt * 10.0f) % 10);

    pkt_clear(p);
    out_prepare_cmd_header(p, 0x0F);
    memcpy(&p->data[AC_HEADER_SIZE], body, 15);
    p->bytes_loaded = AC_HEADER_SIZE + 15 + 2;
    crc_set(p);
    p->msec = millis();
}

static void log_hex_line(const char *prefix, const uint8_t *buf, size_t len)
{
    if (len > 48) {
        len = 48;
    }
    char line[3 * 48 + 1];
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 4 < sizeof(line); i++) {
        pos += (size_t)snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
    }
#if AUX_LOG_ALL_UART_FRAMES
    ESP_LOGI(TAG, "%s (%u B): %s", prefix, (unsigned)len, line);
#else
    ESP_LOGD(TAG, "%s (%u B): %s", prefix, (unsigned)len, line);
#endif
}

/* Last TX for loopback detection (must be before send_out_packet). */
static uint8_t s_last_tx_copy[AC_BUFFER_SIZE];
static uint8_t s_last_tx_copy_len;
static unsigned s_tx_echo_streak;
static uint32_t s_last_echo_hint_ms;
static uint32_t s_last_poll_timeout_log_ms;
static bool s_hw_echo_latch_logged;

static void send_out_packet(void)
{
    if (s_out.bytes_loaded == 0) {
        return;
    }
    if (s_out.bytes_loaded <= sizeof(s_last_tx_copy)) {
        memcpy(s_last_tx_copy, s_out.data, s_out.bytes_loaded);
        s_last_tx_copy_len = s_out.bytes_loaded;
    } else {
        s_last_tx_copy_len = 0;
    }
    log_hex_line("TX AUX", s_out.data, s_out.bytes_loaded);
    uart_write_bytes(HLINK_UART_NUM, s_out.data, s_out.bytes_loaded);
    uart_wait_tx_done(HLINK_UART_NUM, pdMS_TO_TICKS(100));
    pkt_clear(&s_out);
}

static void fill_ping_reply(aux_packet_t *p)
{
    pkt_clear(p);
    p->data[0] = AC_PACKET_START_BYTE;
    p->data[1] = 0;
    p->data[2] = AC_PTYPE_PING;
    p->data[3] = AC_PACKET_ANSWER;
    p->data[4] = 0x01;
    p->data[5] = 0;
    p->data[6] = 8;
    p->data[7] = 0;
    p->data[8] = 0x1C;
    p->data[9] = 0x27;
    p->data[10] = 0;
    p->data[11] = 0;
    p->data[12] = 0;
    p->data[13] = 0;
    p->data[14] = 0;
    p->data[15] = 0;
    p->bytes_loaded = AC_HEADER_SIZE + 8 + 2;
    crc_set(p);
    p->msec = millis();
}

/* Status poll: ESPHome waits for INFO 0x07 (body 0x0F) before requesting big — we sent both in ~1 ms. */
enum { POLL_IDLE = 0, POLL_SEND_SMALL = 1, POLL_WAIT_SMALL = 2, POLL_SEND_BIG = 3, POLL_WAIT_BIG = 4 };
static uint8_t s_poll_step;
static uint32_t s_poll_sent_ms;
static bool s_user_cmd_pending;
static uint32_t s_last_no_status_warn_ms;

#define POLL_RESPONSE_TIMEOUT_MS 600
#define POLL_SLOW_AFTER_ECHO_MS 45000U /* avoid USB flood + UART spam when wiring is wrong */
#define ECHO_HINT_INTERVAL_MS   120000U

static void parse_incoming(void)
{
    if (!crc_ok(&s_in)) {
        ESP_LOGW(TAG, "RX CRC fail");
        log_hex_line("RX bad CRC", s_in.data, s_in.bytes_loaded);
        return;
    }
    log_hex_line("RX AUX", s_in.data, s_in.bytes_loaded);
    uint8_t ptype = s_in.data[2];
    uint8_t bl = s_in.data[6];

    if (ptype == AC_PTYPE_PING && bl == 0) {
        s_has_connection = true; /* confirmed bidirectional */
        fill_ping_reply(&s_out);
        s_acsm = ACSM_SENDING_PACKET;
        return;
    }

    /* HVAC should answer with 0x07 INFO; 0x06 + 2B body matching last TX = local loopback, not the AC. */
    if (ptype == AC_PTYPE_CMD && bl == 2) {
        bool exact_echo = (s_last_tx_copy_len > 0 && s_in.bytes_loaded == s_last_tx_copy_len
                           && memcmp(s_in.data, s_last_tx_copy, s_last_tx_copy_len) == 0);
        if (exact_echo) {
            s_tx_echo_streak++;
        }
        uint32_t now = millis();
        if (exact_echo && s_tx_echo_streak >= 3 && !s_hw_echo_latch_logged) {
            s_hw_echo_latch_logged = true;
            ESP_LOGE(TAG,
                     "Hardware: RX frame == last TX (loopback). Swap TX/RX, fix level shifter GND, "
                     "use AUX indoor Wi‑Fi module UART only (4800 8E1). Software cannot fix this.");
        }
        if (s_last_echo_hint_ms == 0u || (now - s_last_echo_hint_ms) >= ECHO_HINT_INTERVAL_MS) {
            s_last_echo_hint_ms = now;
            if (exact_echo) {
                ESP_LOGW(TAG,
                         "RX == last TX (UART loopback). Indoor MCU not on this RX line — swap TX/RX, check wiring.");
            } else {
                ESP_LOGW(TAG, "RX 0x06 short cmd (subcmd %02X); expect 0x07 INFO (15 B body) from AC",
                         s_in.data[AC_HEADER_SIZE]);
            }
        }
        return;
    }

    if (ptype != AC_PTYPE_INFO || bl < 2 || (size_t)bl + AC_HEADER_SIZE + 2 > sizeof(s_in.data)) {
        return;
    }
    const uint8_t *body = &s_in.data[AC_HEADER_SIZE];
    if (body[0] != 0x01) {
        return;
    }

    switch (body[1]) {
        case AC_CMD_STATUS_SMALL:
            if (bl < 15) {
                ESP_LOGW(TAG, "INFO small: body_len=%u (need 15 / 0x0F)", (unsigned)bl);
                break;
            }
            if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                parse_small_body(body, bl);
                s_tx_echo_streak = 0;
                s_hw_echo_latch_logged = false;
                if (s_poll_step == POLL_WAIT_SMALL) {
                    s_poll_step = POLL_SEND_BIG;
                }
                xSemaphoreGive(s_state_mutex);
            }
            break;
        case AC_CMD_STATUS_BIG:
        case AC_CMD_STATUS_PERIODIC:
            if (bl < 24) {
                break;
            }
            if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                parse_big_body(body, bl);
                s_tx_echo_streak = 0;
                if (s_poll_step == POLL_WAIT_BIG) {
                    s_poll_step = POLL_IDLE;
                }
                xSemaphoreGive(s_state_mutex);
            }
            break;
        default:
            break;
    }

    if (s_state_cb && s_have_status) {
        s_state_cb();
    }
}

static void uart_rx_append(uint8_t byte)
{
    if (s_in.bytes_loaded >= AC_BUFFER_SIZE) {
        pkt_clear(&s_in);
        return;
    }
    if (s_in.bytes_loaded == 0 && byte != AC_PACKET_START_BYTE) {
        return;
    }
    if (s_in.bytes_loaded == 0) {
        s_in.msec = millis();
    }
    s_in.data[s_in.bytes_loaded++] = byte;

    if (s_in.bytes_loaded == AC_HEADER_SIZE) {
        uint8_t bl = s_in.data[6];
        if (bl > AC_BUFFER_SIZE - AC_HEADER_SIZE - 2) {
            pkt_clear(&s_in);
        }
    }
    if (s_in.bytes_loaded >= AC_HEADER_SIZE) {
        uint8_t bl = s_in.data[6];
        uint16_t need = (uint16_t)(AC_HEADER_SIZE + bl + 2);
        if (s_in.bytes_loaded == need) {
            s_acsm = ACSM_PARSING_PACKET;
        }
    }
}

static void uart_task(void *arg)
{
    (void)arg;
    s_last_poll_ms = millis();

    for (;;) {
        switch (s_acsm) {
            case ACSM_SENDING_PACKET:
                send_out_packet();
                s_acsm = ACSM_IDLE;
                break;

            case ACSM_PARSING_PACKET:
                parse_incoming();
                pkt_clear(&s_in);
                s_acsm = ACSM_IDLE;
                break;

            case ACSM_RECEIVING_PACKET: {
                uint8_t b[32];
                int n = uart_read_bytes(HLINK_UART_NUM, b, sizeof(b), 0);
                for (int i = 0; i < n; i++) {
                    uart_rx_append(b[i]);
                    if (s_acsm == ACSM_PARSING_PACKET) {
                        break;
                    }
                }
                if (s_acsm == ACSM_RECEIVING_PACKET && s_in.bytes_loaded > 0
                    && (millis() - s_in.msec) > s_packet_timeout_ms) {
                    ESP_LOGW(TAG, "Packet RX timeout");
                    pkt_clear(&s_in);
                    s_acsm = ACSM_IDLE;
                }
                break;
            }

            case ACSM_IDLE:
            default: {
                size_t avail = 0;
                uart_get_buffered_data_len(HLINK_UART_NUM, &avail);
                if (avail > 0) {
                    uint8_t peek = 0;
                    uart_read_bytes(HLINK_UART_NUM, &peek, 1, 0);
                    if (peek == AC_PACKET_START_BYTE) {
                        s_in.data[0] = peek;
                        s_in.bytes_loaded = 1;
                        s_in.msec = millis();
                        s_acsm = ACSM_RECEIVING_PACKET;
                    }
                } else {
                    if (s_user_cmd_pending && s_out.bytes_loaded == 0 && s_has_connection) {
                        if (xSemaphoreTake(s_state_mutex, 0) == pdTRUE) {
                            if (!s_have_status) {
                                uint32_t nw = millis();
                                if (nw - s_last_no_status_warn_ms > 5000) {
                                    ESP_LOGW(TAG, "Command from Zigbee queued; no AUX status yet (check UART to indoor unit)");
                                    s_last_no_status_warn_ms = nw;
                                }
                            } else {
                                fill_set_params(&s_out);
                                if (s_out.bytes_loaded > 0) {
                                    s_user_cmd_pending = false;
                                    s_acsm = ACSM_SENDING_PACKET;
                                } else {
                                    ESP_LOGW(TAG, "fill_set_params produced no packet");
                                }
                            }
                            xSemaphoreGive(s_state_mutex);
                        }
                    } else if (s_poll_step == POLL_SEND_SMALL && s_out.bytes_loaded == 0 && s_has_connection) {
                        fill_status_small(&s_out);
                        s_acsm = ACSM_SENDING_PACKET;
                        s_poll_step = POLL_WAIT_SMALL;
                        s_poll_sent_ms = millis();
                    } else if (s_poll_step == POLL_SEND_BIG && s_out.bytes_loaded == 0 && s_has_connection) {
                        fill_status_big(&s_out);
                        s_acsm = ACSM_SENDING_PACKET;
                        s_poll_step = POLL_WAIT_BIG;
                        s_poll_sent_ms = millis();
                    } else if ((millis() - s_last_poll_ms) > HLINK_STATUS_UPDATE_INTERVAL_MS && s_out.bytes_loaded == 0
                               && s_has_connection && s_poll_step == POLL_IDLE) {
                        s_last_poll_ms = millis();
                        s_poll_step = POLL_SEND_SMALL;
                    } else if ((s_poll_step == POLL_WAIT_SMALL || s_poll_step == POLL_WAIT_BIG)
                               && s_out.bytes_loaded == 0) {
                        uint32_t need_ms = (s_tx_echo_streak >= 3) ? POLL_SLOW_AFTER_ECHO_MS : POLL_RESPONSE_TIMEOUT_MS;
                        if ((millis() - s_poll_sent_ms) > need_ms) {
                            uint32_t nw = millis();
                            if (nw - s_last_poll_timeout_log_ms > 60000u) {
                                s_last_poll_timeout_log_ms = nw;
                                ESP_LOGW(TAG, "AUX status poll timeout (step=%u, echo_streak=%u) — retry small",
                                         (unsigned)s_poll_step, s_tx_echo_streak);
                            }
                            s_poll_step = POLL_SEND_SMALL;
                            s_poll_sent_ms = millis();
                        }
                    }
                }
                break;
            }
        }
        if (s_acsm == ACSM_IDLE && s_out.bytes_loaded > 0 && s_out.msec > 0) {
            s_acsm = ACSM_SENDING_PACKET;
        }
        /* Always block ≥1 tick: pdMS_TO_TICKS(1) can be 0 at 100Hz and starve IDLE → task WDT */
        vTaskDelay(1);
    }
}

static void queue_user_set(void)
{
    s_user_cmd_pending = true;
}

esp_err_t hlink_driver_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    memset(&s_zb, 0, sizeof(s_zb));
    s_zb.mode = HLINK_ZB_MODE_AUTO;
    s_zb.target_temperature = 22.0f;
    s_zb.current_temperature = 20.0f;

    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) {
        return ESP_FAIL;
    }

    uart_config_t cfg = {
        .baud_rate = HLINK_UART_BAUD_RATE,
        .data_bits = HLINK_UART_DATA_BITS,
        .parity = HLINK_UART_PARITY,
        .stop_bits = HLINK_UART_STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(HLINK_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(HLINK_UART_NUM, HLINK_UART_TX_PIN, HLINK_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(HLINK_UART_NUM, HLINK_UART_BUF_SIZE, HLINK_UART_BUF_SIZE, 0, NULL, 0));
    uart_flush_input(HLINK_UART_NUM);

    pkt_clear(&s_in);
    pkt_clear(&s_out);
    s_acsm = ACSM_IDLE;

    if (xTaskCreate(uart_task, "aux_uart", 4096, NULL, 5, &s_uart_task) != pdPASS) {
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "AUX UART: port %d TX=%d RX=%d %d 8E1 (poll starts without PING)", HLINK_UART_NUM,
             HLINK_UART_TX_PIN, HLINK_UART_RX_PIN, HLINK_UART_BAUD_RATE);
    ESP_LOGI(TAG, "Debug: idf.py menuconfig → Log → Default level DEBUG to print each AUX frame (AUX_DRV).");
    return ESP_OK;
}

esp_err_t hlink_get_state(hlink_state_t *state)
{
    if (!state || !s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    memcpy(state, &s_zb, sizeof(hlink_state_t));
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

void hlink_register_state_change_callback(void (*callback)(void))
{
    s_state_cb = callback;
}

esp_err_t hlink_set_power(bool power_on)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_zb.power_on = power_on;
    queue_user_set();
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

esp_err_t hlink_set_mode(hlink_zb_mode_t mode)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_zb.mode = mode;
    queue_user_set();
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

esp_err_t hlink_set_temperature(float temp_c)
{
    if (temp_c < 16.0f || temp_c > 32.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_zb.target_temperature = temp_c;
    queue_user_set();
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

esp_err_t hlink_set_fan_mode(hlink_zb_fan_t fan_mode)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_zb.fan_mode = fan_mode;
    s_fan_user_override_until_ms = millis() + 5000;
    queue_user_set();
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

esp_err_t hlink_set_swing_mode(uint8_t swing_mode)
{
    if (swing_mode > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_zb.swing_mode = swing_mode;
    queue_user_set();
    xSemaphoreGive(s_state_mutex);
    return ESP_OK;
}

esp_err_t hlink_set_remote_lock(bool locked)
{
    (void)locked;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t hlink_set_beeper(bool enabled)
{
    (void)enabled;
    return ESP_OK;
}

esp_err_t hlink_set_leave_home(bool enabled)
{
    (void)enabled;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t hlink_reset_filter_warning(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t hlink_request_status_update(void)
{
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (s_poll_step == POLL_IDLE) {
        s_poll_step = POLL_SEND_SMALL;
    }
    return ESP_OK;
}

esp_err_t hlink_read_model_name(void)
{
    s_zb.model_name[0] = '\0';
    return ESP_ERR_NOT_SUPPORTED;
}

bool hlink_is_ready(void)
{
    return s_initialized;
}

void hlink_bus_diagnostics(uint8_t duration_sec)
{
    if (duration_sec < 1) {
        duration_sec = 1;
    }
    ESP_LOGI(TAG, "AUX diagnostics: UART active, binary protocol 0xBB..., %d baud 8E1", HLINK_UART_BAUD_RATE);
    uint32_t t0 = millis();
    int total = 0;
    while ((millis() - t0) < (uint32_t)duration_sec * 1000U) {
        uint8_t b;
        if (uart_read_bytes(HLINK_UART_NUM, &b, 1, pdMS_TO_TICKS(100)) > 0) {
            total++;
        }
    }
    ESP_LOGI(TAG, "Bytes seen in %u s: %d", (unsigned)duration_sec, total);
}

void hlink_probe_gpio_levels(uint8_t duration_sec)
{
    (void)duration_sec;
    ESP_LOGW(TAG, "GPIO probe skipped (would conflict with UART)");
}
