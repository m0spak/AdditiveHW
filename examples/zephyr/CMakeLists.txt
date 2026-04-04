cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ahw_example)

target_sources(app PRIVATE
    main.c
    ../../src/additive_hw_api.cpp
)

target_include_directories(app PRIVATE
    ../../src
)
