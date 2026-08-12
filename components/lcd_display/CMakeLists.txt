idf_component_register(
    SRCS "lcd_display.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver esp_driver_spi esp_driver_gpio esp_timer crid_common
)
