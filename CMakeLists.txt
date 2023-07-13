# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(CONF_FILE prj.conf)
set(BOARD rpi_pico)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(minimal)
MESSAGE(“$ENV{CMAKE_CXXFLAGS}”)
target_sources(app PRIVATE src/main.cpp src/MiniPID.cpp src/Motor.cpp  src/Light_sensor.cpp src/CCD_sensor.cpp)