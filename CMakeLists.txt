cmake_minimum_required(VERSION 3.16)
project(shieldcomm VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SHIELDCOMM_SOURCES
  src/shieldcomm.cpp
  common/uart_protocol.c
)

add_library(shieldcomm SHARED ${SHIELDCOMM_SOURCES})
add_library(shieldcomm_static STATIC ${SHIELDCOMM_SOURCES})
set_target_properties(shieldcomm_static PROPERTIES OUTPUT_NAME shieldcomm)
set_target_properties(shieldcomm PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(shieldcomm PUBLIC
  include
  common
)
target_include_directories(shieldcomm_static PUBLIC
  include
  common
)
