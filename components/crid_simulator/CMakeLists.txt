# crid_simulator 组件 — v2.5.12 临时禁用以隔离侦测 RID bug
# 当侦测 bug 修复后恢复：
#   SRCS sim_core.c sim_encode.c sim_wifi.c sim_patrol.c sim_lcd_ui.c
idf_component_register(
    SRCS
    INCLUDE_DIRS "include"
    REQUIRES
)
