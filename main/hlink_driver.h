/*
 * UART driver for AUX-family HVAC (AUX / Kentatsu / Ballu / etc. Wi‑Fi module protocol)
 * Protocol: https://github.com/GrKoR/AUX_HVAC_Protocol
 * ESPHome reference: https://github.com/GrKoR/esphome_aux_ac_component
 *
 * Public API keeps original hlink_* names so the Zigbee layer is unchanged.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HVAC Mode enumeration (Zigbee mapping — unchanged) */
typedef enum {
    HLINK_ZB_MODE_OFF = 0,
    HLINK_ZB_MODE_AUTO = 1,
    HLINK_ZB_MODE_COOL = 3,
    HLINK_ZB_MODE_HEAT = 4,
    HLINK_ZB_MODE_DRY = 8,
    HLINK_ZB_MODE_FAN_ONLY = 7
} hlink_zb_mode_t;

/* HVAC Fan speed enumeration (Zigbee) */
typedef enum {
    HLINK_ZB_FAN_AUTO = 0,
    HLINK_ZB_FAN_HIGH = 1,
    HLINK_ZB_FAN_MEDIUM = 2,
    HLINK_ZB_FAN_LOW = 3,
    HLINK_ZB_FAN_QUIET = 4
} hlink_zb_fan_t;

/* HVAC Running State enumeration */
typedef enum {
    HLINK_RUNNING_IDLE = 0,
    HLINK_RUNNING_HEATING = 1,
    HLINK_RUNNING_COOLING = 2,
    HLINK_RUNNING_FAN_ONLY = 3,
    HLINK_RUNNING_DRYING = 4
} hlink_running_state_t;

/* Aggregated state for Zigbee bridge */
typedef struct {
    bool power_on;
    hlink_zb_mode_t mode;
    hlink_running_state_t running_state;
    uint16_t raw_ac_mode;               /* AUX mode nibble (AC_MODE_* mirror) */
    float current_temperature;          /* Indoor (°C) */
    float outdoor_temperature;          /* Outdoor (°C), from big status */
    float target_temperature;           /* Setpoint (°C) */
    hlink_zb_fan_t fan_mode;
    uint8_t swing_mode;                 /* 0=off, 1=vert, 2=horiz, 3=both */
    bool remote_lock;                   /* Not supported on AUX UART — always false */
    bool beeper_enabled;                /* Not used */
    bool leave_home_enabled;            /* Not supported — always false */
    bool filter_warning;                /* Not supported — always false */
    bool activity_status;               /* Not used */
    char model_name[32];               /* Not queried — empty */
} hlink_state_t;

/* UART: AUX Wi‑Fi module bus is 4800 8E1. If RX bytes == TX bytes (loopback), swap TX/RX pins below. */
#define HLINK_UART_NUM           UART_NUM_1
#define HLINK_UART_TX_PIN        5
#define HLINK_UART_RX_PIN        4
#define HLINK_UART_BAUD_RATE     4800
#define HLINK_UART_PARITY        UART_PARITY_EVEN
#define HLINK_UART_STOP_BITS     UART_STOP_BITS_1
#define HLINK_UART_DATA_BITS     UART_DATA_8_BITS
#define HLINK_UART_BUF_SIZE      2048

#define HLINK_STATUS_UPDATE_INTERVAL_MS 7000
#define HLINK_PACKET_TIMEOUT_MS         150
#define HLINK_POLL_INTERVAL_DEFAULT_MS  30000

typedef enum {
    HLINK_FRAME_NOTHING = 0,
    HLINK_FRAME_PARTIAL = 1,
    HLINK_FRAME_OK = 2,
    HLINK_FRAME_NG = 3,
    HLINK_FRAME_INVALID = 4
} hlink_frame_status_t;

esp_err_t hlink_driver_init(void);
esp_err_t hlink_get_state(hlink_state_t *state);
void hlink_register_state_change_callback(void (*callback)(void));

esp_err_t hlink_set_power(bool power_on);
esp_err_t hlink_set_mode(hlink_zb_mode_t mode);
esp_err_t hlink_set_temperature(float temp_c);
esp_err_t hlink_set_fan_mode(hlink_zb_fan_t fan_mode);
esp_err_t hlink_set_swing_mode(uint8_t swing_mode);

esp_err_t hlink_set_remote_lock(bool locked);
esp_err_t hlink_set_beeper(bool enabled);
esp_err_t hlink_set_leave_home(bool enabled);
esp_err_t hlink_reset_filter_warning(void);

esp_err_t hlink_request_status_update(void);
esp_err_t hlink_read_model_name(void);
bool hlink_is_ready(void);

void hlink_bus_diagnostics(uint8_t duration_sec);
void hlink_probe_gpio_levels(uint8_t duration_sec);

#ifdef __cplusplus
}
#endif
