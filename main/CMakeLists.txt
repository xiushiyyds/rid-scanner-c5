# main/CMakeLists.txt — Wrapper component for main_rx (detector variant)
set(MAIN_RX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../main_rx)

idf_component_register(
    SRCS
        "${MAIN_RX_DIR}/app_main.c"
        "${MAIN_RX_DIR}/crid_sniffer.c"
        "${MAIN_RX_DIR}/crid_parser_common.c"
        "${MAIN_RX_DIR}/crid_parser_gb46750.c"
        "${MAIN_RX_DIR}/crid_parser_astm.c"
        "${MAIN_RX_DIR}/crid_parser_gb42590.c"
        "${MAIN_RX_DIR}/crid_tracker.c"
        "${MAIN_RX_DIR}/crid_display.c"
        "${MAIN_RX_DIR}/crid_json.c"
        "${MAIN_RX_DIR}/crid_ble.c"
        "${MAIN_RX_DIR}/crid_ble_scan.c"
        "${MAIN_RX_DIR}/geofence.c"
        "${MAIN_RX_DIR}/gps_module.c"
        "${MAIN_RX_DIR}/crid_usb_net.c"
        "${MAIN_RX_DIR}/dji_droneid.c"
        "${MAIN_RX_DIR}/evlog.c"
    INCLUDE_DIRS "${MAIN_RX_DIR}"
    PRIV_REQUIRES opendroneid bt lcd_display crid_common crid_simulator
    REQUIRES nvs_flash esp_wifi esp_netif esp_event log
             esp_driver_uart esp_hw_support esp_system esp_timer
             spi_flash esp_driver_gpio
)
