cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(radar_unit_tests)

target_sources(app PRIVATE ../../src/radar_logic.c test_radar.c)
target_include_directories(app PRIVATE ../../include)
