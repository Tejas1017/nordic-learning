cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(spi)

target_sources(app PRIVATE src/main.c)
# target_sources(app PRIVATE src/adxl362.c)
# target_sources(app PRIVATE src/adxl362.h)
# zephyr_library_sources(adxl362.c)
# zephyr_library_sources_ifdef(CONFIG_ADXL362_TRIGGER adxl362_trigger.c)