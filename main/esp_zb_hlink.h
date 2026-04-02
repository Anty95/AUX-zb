/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee H-Link HVAC Thermostat Header
 */

#ifndef ESP_ZB_HLINK_H
#define ESP_ZB_HLINK_H

#include "esp_zigbee_core.h"
#include "hlink_driver.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN        /* aging timeout of device */
#define ED_KEEP_ALIVE                   3000                                 /* 3000 millisecond */
#define HA_HLINK_THERMOSTAT_ENDPOINT    1                                    /* HVAC thermostat endpoint */
#define HA_HLINK_SWING_ENDPOINT         2                                    /* Swing mode switch endpoint */
#define HA_HLINK_REMOTE_LOCK_ENDPOINT   3                                    /* Remote control lock switch endpoint */
#define HA_HLINK_BEEPER_ENDPOINT        4                                    /* Beeper switch endpoint */
#define HA_HLINK_LEAVE_HOME_ENDPOINT    5                                    /* Leave home (away) preset switch endpoint */
#define HA_HLINK_FILTER_WARNING_ENDPOINT 6                                   /* Air filter warning binary sensor endpoint */
#define HA_HLINK_FAN_ENDPOINT           7                                    /* Fan Control only — visible in ZHA cluster UI */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Button configuration */
#define ESP_INTR_FLAG_DEFAULT 0

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x14""Custom devices (DiY)"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x08""hlink-zb"                   /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

/* Function declarations */
void hlink_update_zigbee_attributes(uint8_t param);
void hlink_poll_task(uint8_t param);

#endif
