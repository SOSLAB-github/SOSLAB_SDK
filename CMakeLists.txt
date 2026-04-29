cmake_minimum_required(VERSION 3.10)

project(SOSLAB_SDK VERSION 1.0.1)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/_archive_")

set(CMAKE_INSTALL_BINDIR "bin")
set(CMAKE_INSTALL_LIBDIR "lib")
set(CMAKE_INSTALL_INCLUDEDIR "include")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(API_SOURCE_DIR ${CMAKE_SOURCE_DIR}/soslab_api)
set(GUI_SOURCE_DIR ${CMAKE_SOURCE_DIR}/soslab_gui)

add_library(Win32Lib INTERFACE)

if (WIN32)
    target_link_libraries(Win32Lib INTERFACE ws2_32 mswsock)
endif()

set(OUTPUT_ROOT_DIR "${CMAKE_SOURCE_DIR}/_archive_")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_ROOT_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_ROOT_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_ROOT_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH_SUFFIX "_x64")
else()
    set(ARCH_SUFFIX "_x86")
endif()


set(ASIO_INCLUDE_DIR "${API_SOURCE_DIR}/thirdparty/asio1.30.2/")
set(NLOHMANN_JSON_INCLUDE_DIR "${API_SOURCE_DIR}/thirdparty/nlohmann_json/")
set(UTILL_INCLUDE_DIR "${API_SOURCE_DIR}/internal/utill/")
set(INCLUDE_DIR "${API_SOURCE_DIR}/include/")

# Version header
configure_file(${INCLUDE_DIR}/version.h.in ${INCLUDE_DIR}/version.h @ONLY)

set(LIDAR_LIB_NAME "Lidar${ARCH_SUFFIX}$<$<CONFIG:Debug>:_debug>$<$<CONFIG:Release>:_release>")
message(STATUS "soslab_api: Public Build")
add_subdirectory(soslab_api)
