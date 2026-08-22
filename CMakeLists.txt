cmake_minimum_required(VERSION 3.16)

# ================================================================
# RemoteID Scanner - NekolunaRID 无人机远程识别接收器
# ================================================================
#
# 用法:
#   idf.py --preview set-target esp32c5
#   idf.py --preview build
#
# v1.3+ 结构说明:
#   main/CMakeLists.txt  — 主组件(包装 main_rx 源码)
#   main_rx/             — 主应用源码
#   components/          — 子组件(lcd_display, crid_simulator 等)
# ================================================================

# v2.1.0: PTA 硬件共存 + GB46750 BLE 路径修复 + 删除 TDD
set(CRID_VERSION "2.7.0")

message(STATUS ">>> Building NekolunaRID Scanner v${CRID_VERSION}")

# 添加 components 目录为额外组件
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remoteid_scanner)
