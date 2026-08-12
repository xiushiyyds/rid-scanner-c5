# crid_simulator 组件 — RID 伪造模拟器
# 从 esp32-crid-sim-OTA 项目移植，适配 ESP32-C5

idf_component_register(
    SRCS sim_core.c sim_encode.c sim_wifi.c sim_patrol.c
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash esp_netif esp_event esp_timer driver
             esp_driver_gpio esp_hw_support
    PRIV_REQUIRES soc
)
target_link_libraries(${COMPONENT_LIB} m)
