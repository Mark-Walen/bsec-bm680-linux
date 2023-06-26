cmake_minimum_required(VERSION 2.8)

project(bsec_bme680_basic)

include_directories(
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/bsec_integration
    ${CMAKE_SOURCE_DIR}/bme68x)

set(SHARED_LIB_DRIVER driver)

add_library(${SHARED_LIB_DRIVER} SHARED ${CMAKE_SOURCE_DIR}/common/driver.c)
set_property(TARGET ${SHARED_LIB_DRIVER} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${SHARED_LIB_DRIVER} PRIVATE rt i2c)

set(SOURCES basic.c)
add_executable(${PROJECT_NAME} ${SOURCES})

target_sources(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/bsec_integration/bsec_integration.c ${CMAKE_SOURCE_DIR}/bme68x/bme68x.c)

target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib)

target_link_libraries(${PROJECT_NAME} PRIVATE algobsec m driver)
