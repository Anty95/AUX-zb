/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee H-Link Bridge Application
 * 
 * Adapts Zigbee HA Thermostat commands to AUX UART HVAC protocol (Wi‑Fi module serial).
 * Based on esp_zb_hvac.c from ACW02 project.
 */

#include "esp_zb_hlink.h"
#include "hlink_driver.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_on_off.h"
#include "zcl/esp_zigbee_zcl_fan_control.h"
#include "zcl/esp_zigbee_zcl_thermostat.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "board.h"
#include "version.h"

/* Tag for logging */
static const char *TAG = "ESP_ZB_HLINK";

/* Boot button for factory reset */
#define BOOT_BUTTON_GPIO    GPIO_NUM_9
#define BUTTON_HOLD_TIME_MS 5000

/* H-Link polling interval - query device status every 30 seconds */
#define HLINK_POLL_INTERVAL_MS 30000

/* Factory reset state tracking */
static bool button_pressed = false;
static int64_t button_press_time = 0;

/**
 * Fill ZCL string attribute (first byte = length, followed by string data)
 * For strings longer than max_len-1, truncates and ensures null termination
 */
static void fill_zcl_string(char *dest, size_t max_len, const char *src)
{
    if (!dest || !src || max_len < 2) return;
    
    size_t src_len = strlen(src);
    size_t copy_len = (src_len < max_len - 1) ? src_len : max_len - 2;
    
    dest[0] = (char)copy_len;  // First byte = length
    memcpy(dest + 1, src, copy_len);
    dest[copy_len + 1] = '\0';  // Null terminate for safety
}

/* ZHA names secondary switches from Basic Model Identifier when present */
static void basic_add_model_and_mfg(esp_zb_attribute_list_t *bc, const char *model_id_ascii)
{
    char model_id[16], manufacturer[16];
    uint8_t app_version = (FW_VERSION_MAJOR << 4) | FW_VERSION_MINOR;
    fill_zcl_string(model_id, sizeof(model_id), model_id_ascii);
    fill_zcl_string(manufacturer, sizeof(manufacturer), MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(bc, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);
    esp_zb_basic_cluster_add_attr(bc, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(bc, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
}

/* Boot button interrupt handler */
static void IRAM_ATTR boot_button_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
        /* Button pressed */
        button_pressed = true;
        button_press_time = now;
    } else {
        /* Button released */
        button_pressed = false;
    }
}

/* Boot button monitoring task */
static void boot_button_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[BUTTON] Boot button task started - press and hold for 5 seconds to factory reset");
    
    while (1) {
        if (button_pressed) {
            int64_t now = esp_timer_get_time() / 1000;
            int64_t hold_time = now - button_press_time;
            
            // Log every second to show button is being held
            static int64_t last_log_time = 0;
            if ((hold_time / 1000) > (last_log_time / 1000)) {
                ESP_LOGI(TAG, "[BUTTON] Holding... %lld seconds", hold_time / 1000);
                last_log_time = hold_time;
            }
            
            if (hold_time >= BUTTON_HOLD_TIME_MS) {
                ESP_LOGW(TAG, "[BUTTON] *** FACTORY RESET TRIGGERED ***");
                ESP_LOGW(TAG, "[BUTTON] Erasing Zigbee NVRAM and rebooting...");
                
                /* Factory reset - erase all Zigbee network data */
                esp_zb_factory_reset();
                
                /* Should not reach here - esp_zb_factory_reset() reboots device */
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        } else {
            // Reset button released - clear state
            button_press_time = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Initialize boot button for factory reset */
static esp_err_t button_init(void)
{
    ESP_LOGI(TAG, "[INIT] Configuring boot button on GPIO %d for factory reset...", BOOT_BUTTON_GPIO);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIO");
    
    /* Install ISR service and add handler
     * If ISR service is already installed elsewhere, reuse it instead of failing. */
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(isr_ret, TAG, "Failed to install ISR service");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BOOT_BUTTON_GPIO, boot_button_isr_handler, NULL),
                       TAG, "Failed to add ISR handler");
    
    /* Create button monitoring task */
    xTaskCreate(boot_button_task, "boot_button", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "[OK] Boot button initialization complete");
    return ESP_OK;
}

/* Callback for H-Link state changes - triggers immediate Zigbee update */
static void hlink_state_changed_callback(void)
{
    /* Schedule immediate Zigbee attribute update */
    esp_zb_scheduler_alarm((esp_zb_callback_t)hlink_update_zigbee_attributes, 0, 100);
}

/* Deferred driver initialization (called after Zigbee stack is ready) */
static esp_err_t deferred_driver_init(void)
{
    ESP_LOGI(TAG, "[INIT] Starting deferred driver initialization...");
    
    /* Initialize boot button for factory reset */
    ESP_LOGI(TAG, "[INIT] Initializing boot button...");
    esp_err_t button_ret = button_init();
    if (button_ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Button initialization failed");
        return button_ret;
    }
    ESP_LOGI(TAG, "[INIT] Boot button initialization complete");
    
    /* Small delay to ensure logs are flushed */
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* Initialize H-Link UART driver */
    ESP_LOGI(TAG, "[INIT] Initializing H-Link UART driver...");
    esp_err_t ret = hlink_driver_init();
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ERROR] Failed to initialize H-Link driver: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "[WARN] Continuing without H-Link - endpoints will still be created");
    } else {
        ESP_LOGI(TAG, "[OK] H-Link driver initialized successfully");
        /* Do not read UART here: aux_uart task owns the port; blocking reads caused WDT and races. */
        
        /* Register callback for instant state change notifications */
        ESP_LOGI(TAG, "[INIT] Registering H-Link state change callback...");
        hlink_register_state_change_callback(hlink_state_changed_callback);
        ESP_LOGI(TAG, "[OK] H-Link state change callback registered");
    }
    
    ESP_LOGI(TAG, "[INIT] Deferred initialization complete");
    return ESP_OK;
}

/* Helper function to schedule commissioning */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , 
                       TAG, "Failed to start Zigbee commissioning");
}

/* Zigbee signal handler */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "[JOIN] Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "[JOIN] Device first start - factory new device");
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "[JOIN] Calling deferred driver initialization...");
            esp_err_t init_ret = deferred_driver_init();
            ESP_LOGI(TAG, "[JOIN] Deferred driver initialization %s (ret=%d)", 
                    init_ret == ESP_OK ? "successful" : "failed", init_ret);
            ESP_LOGI(TAG, "[JOIN] Starting network steering...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "[JOIN] Failed to initialize Zigbee stack: %s", esp_err_to_name(err_status));
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "[JOIN] Device reboot - previously joined network");
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "[JOIN] Calling deferred driver initialization...");
            esp_err_t init_ret = deferred_driver_init();
            ESP_LOGI(TAG, "[JOIN] Deferred driver initialization %s", 
                    init_ret == ESP_OK ? "successful" : "failed");
            
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "[JOIN] Factory new - starting network steering...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "[JOIN] Rejoining previous network...");
                esp_zb_ieee_addr_t ieee_addr;
                esp_zb_get_long_address(ieee_addr);
                ESP_LOGI(TAG, "[JOIN] IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                         ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                         ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
            }
        } else {
            ESP_LOGW(TAG, "[JOIN] Failed to initialize Zigbee stack: %s", esp_err_to_name(err_status));
        }
        break;
        
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "[LEAVE] *** DEVICE LEFT THE NETWORK ***");
        if (err_status == ESP_OK) {
            esp_zb_zdo_signal_leave_params_t *leave_params = 
                (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            
            if (leave_params) {
                ESP_LOGW(TAG, "[LEAVE] Leave type: %s", 
                        leave_params->leave_type == 0 ? "RESET" : "REJOIN");
                
                if (leave_params->leave_type == 0) {
                    ESP_LOGW(TAG, "[LEAVE] Device removed from coordinator, scheduling rejoin in 30s...");
                    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                          ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
                }
            }
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "[JOIN] Steering signal received: %s", esp_err_to_name(err_status));
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "[JOIN] *** SUCCESSFULLY JOINED NETWORK ***");
            ESP_LOGI(TAG, "[JOIN] Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0]);
            ESP_LOGI(TAG, "[JOIN] PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            
            ESP_LOGI(TAG, "[JOIN] Device is now online and ready");
            
            /* Start polling task to query H-Link status */
            ESP_LOGI(TAG, "[JOIN] Starting H-Link polling task...");
            hlink_request_status_update();  // Initial request
            esp_zb_scheduler_alarm((esp_zb_callback_t)hlink_poll_task, 0, 5000);
            ESP_LOGI(TAG, "[JOIN] Setup complete!");
        } else {
            ESP_LOGW(TAG, "[JOIN] Network steering failed: %s", esp_err_to_name(err_status));
            ESP_LOGI(TAG, "[JOIN] Retrying in 1 second...");
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                  ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        
    default:
        ESP_LOGD(TAG, "[ZB] Unhandled signal: %lu, status: %s", 
                sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* Convert Zigbee system mode to H-Link mode */
static hlink_zb_mode_t zigbee_to_hlink_mode(uint8_t zigbee_mode)
{
    switch (zigbee_mode) {
        case 0x00: return HLINK_ZB_MODE_OFF;      // Off
        case 0x01: return HLINK_ZB_MODE_AUTO;     // Auto
        case 0x03: return HLINK_ZB_MODE_COOL;     // Cool
        case 0x04: return HLINK_ZB_MODE_HEAT;     // Heat
        case 0x07: return HLINK_ZB_MODE_FAN_ONLY; // Fan only
        case 0x08: return HLINK_ZB_MODE_DRY;      // Dry/Dehumidify
        default:   return HLINK_ZB_MODE_AUTO;
    }
}

/* Convert H-Link mode to Zigbee system mode */
static uint8_t hlink_to_zigbee_mode(hlink_zb_mode_t hlink_mode, bool power_on)
{
    if (!power_on) return 0x00;  // Off
    
    switch (hlink_mode) {
        case HLINK_ZB_MODE_AUTO:     return 0x01;  // Auto
        case HLINK_ZB_MODE_COOL:     return 0x03;  // Cool
        case HLINK_ZB_MODE_HEAT:     return 0x04;  // Heat
        case HLINK_ZB_MODE_FAN_ONLY: return 0x07;  // Fan only
        case HLINK_ZB_MODE_DRY:      return 0x08;  // Dry
        default:                     return 0x01;  // Auto
    }
}

/* Convert Zigbee fan mode to H-Link fan speed */
static hlink_zb_fan_t zigbee_to_hlink_fan(uint8_t zigbee_fan)
{
    switch (zigbee_fan) {
        case 0x00: return HLINK_ZB_FAN_AUTO;   // Off -> Auto
        case 0x01: return HLINK_ZB_FAN_LOW;    // Low
        case 0x02: return HLINK_ZB_FAN_MEDIUM; // Medium
        case 0x03: return HLINK_ZB_FAN_HIGH;   // High
        case 0x04: return HLINK_ZB_FAN_AUTO;   // On/Auto
        case 0x05: return HLINK_ZB_FAN_AUTO;   // Auto
        default:   return HLINK_ZB_FAN_AUTO;
    }
}

/* ZBOSS: cooling setpoint write fails if new cool < stored heat (INVALID_VALUE). One AUX target — shadow SPs. */
#define ZCL_SP_MIN_C100 1600
#define ZCL_SP_MAX_C100 3200

static void zcl_shadow_setpoints_after_mode_write(uint8_t zigbee_system_mode)
{
    int16_t min_h = ZCL_SP_MIN_C100;
    int16_t max_c = ZCL_SP_MAX_C100;
    if (zigbee_system_mode == 0x04) {
        esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                     ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                                     &max_c, false);
    } else {
        esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                     ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                     &min_h, false);
    }
}

/* Thermostat Running Mode (ZCL): Off=0x00, Cool=0x03, Heat=0x04 — invalid values break attribute reporting */
static uint8_t hlink_to_zigbee_running_mode(bool power_on, hlink_zb_mode_t mode)
{
    if (!power_on) {
        return 0x00;
    }
    switch (mode) {
        case HLINK_ZB_MODE_HEAT:
            return 0x04;
        case HLINK_ZB_MODE_COOL:
        case HLINK_ZB_MODE_DRY:
        case HLINK_ZB_MODE_FAN_ONLY:
            return 0x03;
        case HLINK_ZB_MODE_AUTO:
        case HLINK_ZB_MODE_OFF:
        default:
            return 0x03;
    }
}

/* Convert H-Link fan speed to Zigbee fan mode (ZCL: Low=0x01 … High=0x03, Auto=0x05) */
static uint8_t hlink_to_zigbee_fan(hlink_zb_fan_t hlink_fan)
{
    switch (hlink_fan) {
        case HLINK_ZB_FAN_AUTO:   return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO;
        case HLINK_ZB_FAN_LOW:    return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW;
        case HLINK_ZB_FAN_MEDIUM: return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_MEDIUM;
        case HLINK_ZB_FAN_HIGH:   return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_HIGH;
        case HLINK_ZB_FAN_QUIET:  return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_LOW;
        default:                  return ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO;
    }
}

/* Update Zigbee attributes from H-Link state */
void hlink_update_zigbee_attributes(uint8_t param)
{
    (void)param;
    
    hlink_state_t state;
    if (hlink_get_state(&state) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get H-Link state for Zigbee update");
        return;
    }
    
    /* Update Thermostat endpoint */
    int16_t local_temp = (int16_t)(state.current_temperature * 100);
    int16_t target_temp = (int16_t)(state.target_temperature * 100);
    int16_t heat_sp;
    int16_t cool_sp;
    if (state.power_on && state.mode == HLINK_ZB_MODE_HEAT) {
        heat_sp = target_temp;
        cool_sp = ZCL_SP_MAX_C100;
    } else {
        cool_sp = target_temp;
        heat_sp = ZCL_SP_MIN_C100;
    }
    uint8_t system_mode = hlink_to_zigbee_mode(state.mode, state.power_on);
    uint8_t running_mode = hlink_to_zigbee_running_mode(state.power_on, state.mode);
    
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT, 
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                 &local_temp, false);
    
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                                 &heat_sp, false);
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                                 &cool_sp, false);
    
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                                 &system_mode, false);
    
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
                                 &running_mode, false);
    
    /* Fan Control on EP1 and EP7 (mirror — ZHA often only lists fan clearly on a dedicated endpoint) */
    uint8_t zigbee_fan = hlink_to_zigbee_fan(state.fan_mode);
    esp_zb_zcl_set_attribute_val(HA_HLINK_THERMOSTAT_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                                 &zigbee_fan, false);
    esp_zb_zcl_set_attribute_val(HA_HLINK_FAN_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                                 &zigbee_fan, false);
    
    /* Update Swing switch */
    uint8_t swing_on = (state.swing_mode != 0) ? 1 : 0;
    esp_zb_zcl_set_attribute_val(HA_HLINK_SWING_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                 &swing_on, false);
    
    /* Update Remote Lock switch */
    uint8_t remote_lock_val = state.remote_lock ? 1 : 0;
    esp_zb_zcl_set_attribute_val(HA_HLINK_REMOTE_LOCK_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                 &remote_lock_val, false);
    
    /* Update Beeper switch */
    uint8_t beeper_on = state.beeper_enabled ? 1 : 0;
    esp_zb_zcl_set_attribute_val(HA_HLINK_BEEPER_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                 &beeper_on, false);
    
    /* Update Leave Home switch */
    uint8_t leave_home = state.leave_home_enabled ? 1 : 0;
    esp_zb_zcl_set_attribute_val(HA_HLINK_LEAVE_HOME_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                 &leave_home, false);
    
    /* Update Filter Warning sensor */
    uint8_t filter_warning = state.filter_warning ? 1 : 0;
    esp_zb_zcl_set_attribute_val(HA_HLINK_FILTER_WARNING_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                 &filter_warning, false);
    
    ESP_LOGI(TAG, "Updated Zigbee: Power=%d Mode=%d Temp=%.1f°C->%.1f°C Fan=%d Swing=%d",
             state.power_on, state.mode, state.current_temperature, state.target_temperature,
             state.fan_mode, state.swing_mode);
}

/* H-Link polling task */
void hlink_poll_task(uint8_t param)
{
    (void)param;
    
    /* Request current status from H-Link device */
    hlink_request_status_update();
    
    /* Schedule next poll */
    esp_zb_scheduler_alarm((esp_zb_callback_t)hlink_poll_task, 0, HLINK_POLL_INTERVAL_MS);
}

/* Zigbee attribute write handler */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool should_update = false;
    
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
                       "Received message: error status(%d)", message->info.status);
    
    ESP_LOGI(TAG, "[ATTR] Received cluster(0x%x) attribute(0x%x) endpoint(%d) data size(%d)",
             message->info.cluster, message->attribute.id, message->info.dst_endpoint,
             message->attribute.data.size);
    
    /* Handle Thermostat endpoint */
    if (message->info.dst_endpoint == HA_HLINK_THERMOSTAT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
            /* System mode changed */
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID) {
                uint8_t mode = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
                ESP_LOGI(TAG, "[HVAC] System mode changed to: 0x%02X", mode);
                
                if (mode == 0x00) {
                    /* Turn off */
                    hlink_set_power(false);
                } else {
                    /* Turn on and set mode */
                    hlink_set_power(true);
                    uint8_t hlink_mode = zigbee_to_hlink_mode(mode);
                    hlink_set_mode(hlink_mode);
                }
                zcl_shadow_setpoints_after_mode_write(mode);
                should_update = true;
            }
            else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID
                     || message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID) {
                int16_t temp = message->attribute.data.value ? *(int16_t *)message->attribute.data.value : 2200;
                uint8_t temp_c = (uint8_t)(temp / 100);
                ESP_LOGI(TAG, "[HVAC] Target temperature (attr 0x%04x) -> %d°C", (unsigned)message->attribute.id,
                         (int)temp_c);
                hlink_set_temperature((float)temp_c);
                should_update = true;
            }
        }
        /* Fan mode changed */
        else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID) {
                uint8_t fan = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
                ESP_LOGI(TAG, "[HVAC] Fan mode changed to: 0x%02X", fan);
                hlink_zb_fan_t hlink_fan = zigbee_to_hlink_fan(fan);
                hlink_set_fan_mode(hlink_fan);
                should_update = true;
            }
        }
    }
    else if (message->info.dst_endpoint == HA_HLINK_FAN_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL
            && message->attribute.id == ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID) {
            uint8_t fan = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
            ESP_LOGI(TAG, "[FAN EP7] Fan mode changed to: 0x%02X", fan);
            hlink_zb_fan_t hlink_fan = zigbee_to_hlink_fan(fan);
            hlink_set_fan_mode(hlink_fan);
            should_update = true;
        }
    }
    /* Handle Swing endpoint */
    else if (message->info.dst_endpoint == HA_HLINK_SWING_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            uint8_t state = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
            ESP_LOGI(TAG, "[SWING] Swing changed to: %d", state);
            hlink_set_swing_mode(state != 0 ? 1 : 0);  // 0=off, 1=on (vertical)
            should_update = true;
        }
    }
    /* Handle Remote Lock endpoint */
    else if (message->info.dst_endpoint == HA_HLINK_REMOTE_LOCK_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            uint8_t state = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
            ESP_LOGI(TAG, "[LOCK] Remote lock changed to: %d", state);
            hlink_set_remote_lock(state != 0);
            should_update = true;
        }
    }
    /* Handle Beeper endpoint */
    else if (message->info.dst_endpoint == HA_HLINK_BEEPER_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            uint8_t state = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
            ESP_LOGI(TAG, "[BEEPER] Beeper changed to: %d", state);
            hlink_set_beeper(state != 0);
            should_update = true;
        }
    }
    /* Handle Leave Home endpoint */
    else if (message->info.dst_endpoint == HA_HLINK_LEAVE_HOME_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            uint8_t state = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
            ESP_LOGI(TAG, "[LEAVE] Leave home changed to: %d", state);
            hlink_set_leave_home(state != 0);
            should_update = true;
        }
    }
    
    /* Schedule Zigbee attribute update if needed */
    if (should_update) {
        esp_zb_scheduler_alarm((esp_zb_callback_t)hlink_update_zigbee_attributes, 0, 500);
    }
    
    return ret;
}

/* esp_zb_on_off_cluster_create() marks OnOff for reporting; with several HA-bound endpoints ZBOSS can
 * call zb_zcl_send_report_attr_command() with a bad reporting entry (load fault at MEPC). Switches only
 * need read/write; coordinators may poll. */
static esp_zb_attribute_list_t *hlink_on_off_cluster_no_reporting(bool on)
{
    esp_zb_attribute_list_t *list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    uint8_t v = on ? 1u : 0u;
    esp_zb_cluster_add_attr(list, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                            ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &v);
    return list;
}

/* Zigbee action handler */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case 4101: /* ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID — ZCL default response after writes */
        ESP_LOGD(TAG, "[ACTION] ZCL default response");
        break;
    default:
        ESP_LOGW(TAG, "[ACTION] Unhandled callback id: %d", callback_id);
        break;
    }
    
    return ret;
}

/* Zigbee task - initializes stack and creates endpoints */
static void esp_zb_task(void *pvParameters)
{
    ESP_LOGI(TAG, "[START] Starting Zigbee task...");
    
    /* Initialize Zigbee stack as Router */
    ESP_LOGI(TAG, "[INIT] Initializing Zigbee stack as Router...");
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            },
        },
    };
    
    esp_zb_init(&zb_nwk_cfg);
    ESP_LOGI(TAG, "[OK] Zigbee stack initialized");
    
    /* Log TX power */
    int8_t tx_power = 0;
    esp_zb_get_tx_power(&tx_power);
    ESP_LOGI(TAG, "Zigbee TX power: %d dBm", tx_power);
    
    /* Create endpoint list */
    ESP_LOGI(TAG, "[INIT] Creating endpoint list...");
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    
    /* ===== THERMOSTAT ENDPOINT (EP1) ===== */
    ESP_LOGI(TAG, "[EP1] Creating Thermostat endpoint...");
    esp_zb_cluster_list_t *thermostat_clusters = esp_zb_zcl_cluster_list_create();
    
    /* Basic cluster */
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,  // Mains powered
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    
    /* Add version info */
    uint8_t app_version = (FW_VERSION_MAJOR << 4) | FW_VERSION_MINOR;
    uint8_t stack_version = 0x30;  // Stack 3.0
    uint8_t hw_version = 1;
    char model_id[16], manufacturer[16], date_code[16], sw_build[16];
    fill_zcl_string(model_id, sizeof(model_id), "AuxZB-Climate");
    fill_zcl_string(manufacturer, sizeof(manufacturer), MANUFACTURER_NAME);
    fill_zcl_string(date_code, sizeof(date_code), FW_DATE_CODE);
    fill_zcl_string(sw_build, sizeof(sw_build), FW_VERSION);
    
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, manufacturer);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_version);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, date_code);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, sw_build);
    
    esp_zb_cluster_list_add_basic_cluster(thermostat_clusters, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Thermostat cluster: no ESP_ZB_ZCL_ATTR_ACCESS_REPORTING — ZBOSS still iterates reporting list from
     * NVRAM; mismatched entries caused zb_zcl_get_next_reporting_info / send_report_attr null deref. */
    esp_zb_attribute_list_t *thermostat_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT);
    
    int16_t local_temp = 2500;  // 25°C
    int16_t heating_setpoint = ZCL_SP_MIN_C100;
    int16_t cooling_setpoint = ZCL_SP_MAX_C100; /* cool >= heat; first heat/cool writes stay valid */
    uint8_t control_seq = 0x04;  // Cool + Heat
    uint8_t system_mode = 0x00;  // Off
    uint8_t running_mode = 0x00;  // Idle
    
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                           ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                           ESP_ZB_ZCL_ATTR_TYPE_S16,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                           &local_temp);
    
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                           ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
                           ESP_ZB_ZCL_ATTR_TYPE_S16,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                           &heating_setpoint);
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                           ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
                           ESP_ZB_ZCL_ATTR_TYPE_S16,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                           &cooling_setpoint);
    
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                           ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
                           ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                           &system_mode);
    
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &control_seq);
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_RUNNING_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &running_mode);
    
    /* Temperature limits (16-32°C for Hitachi) */
    int16_t min_heat = 1600, max_heat = 3200;
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &min_heat);
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &max_heat);
    int16_t min_cool = 1600, max_cool = 3200;
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &min_cool);
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &max_cool);
    /* Without this, ZBOSS uses a default ~2.5 °C dead band: equal heat/cool setpoints
     * (single target on AUX) get Write Attributes -> INVALID_VALUE (0x87 / 135). */
    int8_t min_setpoint_dead_band = 0;
    esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                            ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_SETPOINT_DEAD_BAND_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_S8,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &min_setpoint_dead_band);
    
    esp_zb_cluster_list_add_thermostat_cluster(thermostat_clusters, thermostat_cluster,
                                               ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Fan Control: manual attrs without REPORTING (default helper enables reporting → same crash as multi On/Off). */
    esp_zb_attribute_list_t *fan_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL);
    uint8_t fan_mode_ep1 = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO;
    uint8_t fan_mode_sequence = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO;
    esp_zb_cluster_add_attr(fan_cluster, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                            ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &fan_mode_ep1);
    esp_zb_cluster_add_attr(fan_cluster, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                            ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &fan_mode_sequence);
    esp_zb_cluster_list_add_fan_control_cluster(thermostat_clusters, fan_cluster,
                                                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Identify cluster */
    esp_zb_cluster_list_add_identify_cluster(thermostat_clusters,
                                             esp_zb_identify_cluster_create(NULL),
                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Add endpoint */
    esp_zb_endpoint_config_t thermostat_ep = {
        .endpoint = HA_HLINK_THERMOSTAT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        /* 0x0300 matches HVAC + optional fan better than bare thermostat 0x0301 for coordinators */
        .app_device_id = ESP_ZB_HA_HEATING_COOLING_UNIT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, thermostat_clusters, thermostat_ep);
    ESP_LOGI(TAG, "[OK] Thermostat endpoint created");
    
    /* ===== SWING ENDPOINT (EP2) ===== */
    ESP_LOGI(TAG, "[EP2] Creating Swing endpoint...");
    esp_zb_cluster_list_t *swing_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *swing_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(swing_basic, "AuxZB-Swing");
        esp_zb_cluster_list_add_basic_cluster(swing_clusters, swing_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    
    esp_zb_attribute_list_t *swing_onoff = hlink_on_off_cluster_no_reporting(false);
    esp_zb_cluster_list_add_on_off_cluster(swing_clusters, swing_onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t swing_ep = {
        .endpoint = HA_HLINK_SWING_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, swing_clusters, swing_ep);
    ESP_LOGI(TAG, "[OK] Swing endpoint created");
    
    /* ===== REMOTE LOCK ENDPOINT (EP3) ===== */
    ESP_LOGI(TAG, "[EP3] Creating Remote Lock endpoint...");
    esp_zb_cluster_list_t *lock_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *lock_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(lock_basic, "AuxZB-Lock");
        esp_zb_cluster_list_add_basic_cluster(lock_clusters, lock_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    esp_zb_attribute_list_t *lock_onoff = hlink_on_off_cluster_no_reporting(false);
    esp_zb_cluster_list_add_on_off_cluster(lock_clusters, lock_onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t lock_ep = {
        .endpoint = HA_HLINK_REMOTE_LOCK_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, lock_clusters, lock_ep);
    ESP_LOGI(TAG, "[OK] Remote Lock endpoint created");
    
    /* ===== BEEPER ENDPOINT (EP4) ===== */
    ESP_LOGI(TAG, "[EP4] Creating Beeper endpoint...");
    esp_zb_cluster_list_t *beeper_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *beeper_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(beeper_basic, "AuxZB-Beep");
        esp_zb_cluster_list_add_basic_cluster(beeper_clusters, beeper_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    esp_zb_attribute_list_t *beeper_onoff = hlink_on_off_cluster_no_reporting(false);
    esp_zb_cluster_list_add_on_off_cluster(beeper_clusters, beeper_onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t beeper_ep = {
        .endpoint = HA_HLINK_BEEPER_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, beeper_clusters, beeper_ep);
    ESP_LOGI(TAG, "[OK] Beeper endpoint created");
    
    /* ===== LEAVE HOME ENDPOINT (EP5) ===== */
    ESP_LOGI(TAG, "[EP5] Creating Leave Home endpoint...");
    esp_zb_cluster_list_t *leave_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *leave_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(leave_basic, "AuxZB-Away");
        esp_zb_cluster_list_add_basic_cluster(leave_clusters, leave_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    esp_zb_attribute_list_t *leave_onoff = hlink_on_off_cluster_no_reporting(false);
    esp_zb_cluster_list_add_on_off_cluster(leave_clusters, leave_onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t leave_ep = {
        .endpoint = HA_HLINK_LEAVE_HOME_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, leave_clusters, leave_ep);
    ESP_LOGI(TAG, "[OK] Leave Home endpoint created");
    
    /* ===== FILTER WARNING ENDPOINT (EP6) ===== */
    ESP_LOGI(TAG, "[EP6] Creating Filter Warning sensor endpoint...");
    esp_zb_cluster_list_t *filter_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *filter_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(filter_basic, "AuxZB-Filter");
        esp_zb_cluster_list_add_basic_cluster(filter_clusters, filter_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    
    /* Binary Input cluster for sensor state */
    esp_zb_attribute_list_t *binary_input = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT);
    uint8_t present_value = 0;
    esp_zb_cluster_add_attr(binary_input, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                           ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                           ESP_ZB_ZCL_ATTR_TYPE_BOOL,
                           ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                           &present_value);
    esp_zb_cluster_list_add_binary_input_cluster(filter_clusters, binary_input,
                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    esp_zb_endpoint_config_t filter_ep = {
        .endpoint = HA_HLINK_FILTER_WARNING_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, filter_clusters, filter_ep);
    ESP_LOGI(TAG, "[OK] Filter Warning endpoint created");
    
    /* ===== FAN CONTROL ENDPOINT (EP7) =====
     * Same FanMode as EP1; ZHA "Manage Zigbee device" often omits 0x0202 on busy EP1 — dedicated EP is visible. */
    ESP_LOGI(TAG, "[EP7] Creating Fan Control endpoint...");
    esp_zb_cluster_list_t *fan_only_clusters = esp_zb_zcl_cluster_list_create();
    {
        esp_zb_attribute_list_t *fan_ep_basic = esp_zb_basic_cluster_create(&basic_cfg);
        basic_add_model_and_mfg(fan_ep_basic, "AuxZB-Fan");
        esp_zb_cluster_list_add_basic_cluster(fan_only_clusters, fan_ep_basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    }
    esp_zb_cluster_list_add_identify_cluster(fan_only_clusters,
                                             esp_zb_identify_cluster_create(NULL),
                                             ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *fan_cluster_ep7 = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL);
    uint8_t fan_mode_ep7 = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_AUTO;
    uint8_t fan_mode_sequence_ep7 = ESP_ZB_ZCL_FAN_CONTROL_FAN_MODE_SEQUENCE_LOW_MED_HIGH_AUTO;
    esp_zb_cluster_add_attr(fan_cluster_ep7, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                            ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &fan_mode_ep7);
    esp_zb_cluster_add_attr(fan_cluster_ep7, ESP_ZB_ZCL_CLUSTER_ID_FAN_CONTROL,
                            ESP_ZB_ZCL_ATTR_FAN_CONTROL_FAN_MODE_SEQUENCE_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                            &fan_mode_sequence_ep7);
    esp_zb_cluster_list_add_fan_control_cluster(fan_only_clusters, fan_cluster_ep7,
                                                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t fan_only_ep = {
        .endpoint = HA_HLINK_FAN_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_HEATING_COOLING_UNIT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, fan_only_clusters, fan_only_ep);
    ESP_LOGI(TAG, "[OK] Fan Control endpoint created");
    
    /* Register device */
    ESP_LOGI(TAG, "[INIT] Registering Zigbee device...");
    esp_zb_device_register(esp_zb_ep_list);
    /* Old NVRAM can retain reporting records that no longer match this data model → zb_zcl_get_next_reporting_info fault. */
    esp_zb_zcl_reset_all_reporting_info();
    
    /* Register action callback */
    esp_zb_core_action_handler_register(zb_action_handler);
    
    /* Set default Zigbee channel mask to avoid specific channels */
    ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK));
    
    ESP_LOGI(TAG, "[OK] Zigbee device registered with %d endpoints", 7);
    ESP_LOGI(TAG, "[START] Starting Zigbee stack...");
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    /* Main loop */
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    /* Start Zigbee task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
