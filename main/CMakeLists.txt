idf_component_register(
    SRC_DIRS  "."
    INCLUDE_DIRS "." "${CMAKE_BINARY_DIR}/generated"
    PRIV_REQUIRES nvs_flash esp_driver_uart esp_timer ieee802154 app_update
                  examples_utils espressif__esp-zigbee-lib
)
