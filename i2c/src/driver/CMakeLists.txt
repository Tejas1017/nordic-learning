zephyr_include_directories(.)
target_sources(app PRIVATE $(CMAKE_CURRENT_SOURCE_DIR)/bme680.c)