idf_component_register(
    SRCS "lcd_display.c" "lcd_font.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver esp_driver_spi esp_driver_gpio esp_timer esp_mm crid_common
)

# 强制 C/C++ 源码按 UTF-8 解析（Windows MSVC/GCC 兼容）
target_compile_options(${COMPONENT_LIB} PRIVATE -finput-charset=UTF-8 -fexec-charset=UTF-8)
