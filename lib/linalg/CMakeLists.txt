cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(linalg STATIC linalg.c)

# Make the headers visible to other targets
target_include_directories(linalg PUBLIC ${CMAKE_CURRENT_LIST_DIR})