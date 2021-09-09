cmake_minimum_required(VERSION 2.9)
project(ClimateChamberControlLib)

if(NOT INSTALL_DIR)
    set(INSTALL_DIR "/usr/local")
endif() # if not BUILD_DIR

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wpedantic")

set(src_files "${CMAKE_CURRENT_SOURCE_DIR}/include/helperFiles/Socket.h"
              "${CMAKE_CURRENT_SOURCE_DIR}/include/helperFiles/Logging.h"
              "${CMAKE_CURRENT_SOURCE_DIR}/include/ClimateChamberWrapper.h"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/ClimateChamberWrapper.cpp"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/Socket.c"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/Logging.c")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(ClimateChamberControlLib SHARED ${src_files})
target_link_libraries(ClimateChamberControlLib pthread)

add_executable(ClimateChamberTest test/Main.cpp)
target_link_libraries(ClimateChamberTest PRIVATE ClimateChamberControlLib)

install(TARGETS ClimateChamberTest ClimateChamberControlLib
        ARCHIVE DESTINATION "${INSTALL_DIR}/lib"
        LIBRARY DESTINATION "${INSTALL_DIR}/lib"
        RUNTIME DESTINATION "${INSTALL_DIR}/bin")