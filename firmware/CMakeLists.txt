cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")

# LVGL uses Kconfig (LV_CONF_SKIP=y) — no lv_conf.h needed

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cc02-esp)
