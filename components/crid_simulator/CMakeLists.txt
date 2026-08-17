# crid_simulator 组件 — RID 伪造模拟器 v2.6.0
idf_component_register(
    SRCS sim_core.c sim_encode.c sim_wifi.c sim_patrol.c sim_lcd_ui.c
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash esp_netif esp_event esp_timer driver
             esp_driver_gpio esp_hw_support lcd_display
    PRIV_REQUIRES soc
)
target_link_libraries(${COMPONENT_LIB} m)
