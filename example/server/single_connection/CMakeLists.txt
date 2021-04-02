cmake_minimum_required(VERSION 3.2)

set(HAVE_FLAG_SEARCH_PATHS_FIRST 0)
project( temp_single_server )
set(CMAKE_C_COMPILER "arm-linux-gnueabihf-clang")

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drv_bme280.c
    ${CMAKE_CURRENT_SOURCE_DIR}/single_server.c
    )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../../
)

