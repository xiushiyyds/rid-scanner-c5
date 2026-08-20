# crid_simulator 组件 — RID 伪造模拟器
#
# 仅当 CONFIG_RID_SCANNER_BUILD_SIMULATOR=y 时编译模拟器源码。
# 侦测构建（默认）不产生任何 .o，从根上避免链接模拟器导致的
# 内存布局变化破坏 WiFi/BLE 共存（v2.5.x bug 根因）。

if(CONFIG_RID_SCANNER_BUILD_SIMULATOR)
    idf_component_register(
        SRCS sim_core.c sim_encode.c sim_encode_gb46750.c sim_wifi.c sim_patrol.c sim_lcd_ui.c
        INCLUDE_DIRS "include"
        REQUIRES esp_wifi nvs_flash esp_netif esp_event esp_timer driver
                 esp_driver_gpio esp_hw_support lcd_display crid_common
        PRIV_REQUIRES soc
    )
    target_link_libraries(${COMPONENT_LIB} m)
else()
    idf_component_register(
        SRCS
        INCLUDE_DIRS "include"
        REQUIRES
    )
endif()
