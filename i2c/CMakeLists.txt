cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(i2c)

target_sources(app PRIVATE src/main.c)
add_subdirectory_ifdef(CONFIG_BME680 src/driver/bme680.c)
add_subdirectory_ifdef(CONFIG_BME680 src/driver/bme680_spi.c)
add_subdirectory_ifdef(CONFIG_680 src/driver/bme680.h)