cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(p1fats)


add_subdirectory(src/fatfs)
add_subdirectory(src/modules)
add_subdirectory(src/dhara)




include_directories(src)
include_directories(src/fatfs)
include_directories(src/dhara)
include_directories(src/modules)


target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/mt29f_nand_flash.c)
target_sources(app PRIVATE src/mt29f_nand_flash_reg.c)
