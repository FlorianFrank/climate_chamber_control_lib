cmake_minimum_required(VERSION 3.0)

project(py_climate_chamber_lib CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(pybind11)
pybind11_add_module(py_climate_chamber_lib pybind_wrapper.cpp)

target_compile_definitions(py_climate_chamber_lib PRIVATE VERSION_INFO="${EXAMPLE_VERSION_INFO}")

target_include_directories(py_climate_chamber_lib PRIVATE   ${CMAKE_CURRENT_SOURCE_DIR}
                                                            ${CMAKE_CURRENT_SOURCE_DIR}/../include
                                                            ${CMAKE_CURRENT_SOURCE_DIR}/../common_tools_lib/ErrorHandling/include
                                                            ${CMAKE_CURRENT_SOURCE_DIR}/../common_tools_lib/Logger/include)

target_link_libraries(py_climate_chamber_lib PRIVATE climate_chamber_lib)

install(TARGETS py_climate_chamber_lib
        ARCHIVE DESTINATION "${INSTALL_DIR}/lib"
        LIBRARY DESTINATION "${INSTALL_DIR}/lib"
        RUNTIME DESTINATION "${INSTALL_DIR}/bin")