cmake_minimum_required(VERSION 2.9)

include(cmake/build_config.cmake)
add_definitions(-DCXX=1)

project(ClimateChamberControlLib)

if(NOT INSTALL_DIR)
    set(INSTALL_DIR "/usr/local")
endif() # if not BUILD_DIR

add_subdirectory(common_tools_lib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wpedantic")

set(src_files "${CMAKE_CURRENT_SOURCE_DIR}/include/ClimateChamberWrapper.h"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/ClimateChamberWrapper.cpp")



add_library(ClimateChamberControlLib SHARED ${src_files})

target_include_directories(ClimateChamberControlLib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Communication/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Additional"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Logging/include"
                                                            "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/ErrorHandling/include")


target_link_libraries(ClimateChamberControlLib common_tools_lib pthread common_tools_lib_static)

add_executable(ClimateChamberTest test/Main.cpp)

target_include_directories(ClimateChamberTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Communication/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Additional"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/Logging/include"
                                                      "${CMAKE_CURRENT_SOURCE_DIR}/common_tools_lib/ErrorHandling/include")

target_link_libraries(ClimateChamberTest PRIVATE ClimateChamberControlLib)

install(TARGETS ClimateChamberTest ClimateChamberControlLib
        ARCHIVE DESTINATION "${INSTALL_DIR}/lib"
        LIBRARY DESTINATION "${INSTALL_DIR}/lib"
        RUNTIME DESTINATION "${INSTALL_DIR}/bin")