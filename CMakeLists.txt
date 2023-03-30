cmake_minimum_required(VERSION 3.15)

include(cmake/build_config.cmake)
add_definitions(-DCXX=1)
set(CMAKE_CXX_STANDARD 14)

project(climate_chamber_lib VERSION 1.0.0)

if(NOT INSTALL_DIR)
    set(INSTALL_DIR "/usr/local")
endif() # if not BUILD_DIR

add_subdirectory(common_tools_lib)
add_subdirectory(python_interface)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wpedantic")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wpedantic")

set(src_files "${CMAKE_CURRENT_SOURCE_DIR}/include/ClimateChamberControl.h"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/ClimateChamberControl.cpp")



add_library(climate_chamber_lib SHARED ${src_files})

target_include_directories(climate_chamber_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Threading/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Communication/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Additional"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Logging/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/ErrorHandling/include")


target_link_libraries(climate_chamber_lib common_tools_lib pthread common_tools_lib_static)

add_executable(ClimateChamberTest test/Main.cpp)

target_include_directories(ClimateChamberTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Threading/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Communication/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Additional"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Logging/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/ErrorHandling/include")

target_link_libraries(ClimateChamberTest PRIVATE climate_chamber_lib)

install(TARGETS ClimateChamberTest climate_chamber_lib
        ARCHIVE DESTINATION "${INSTALL_DIR}/lib"
        LIBRARY DESTINATION "${INSTALL_DIR}/lib"
        RUNTIME DESTINATION "${INSTALL_DIR}/bin")
