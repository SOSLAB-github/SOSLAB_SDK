cmake_minimum_required(VERSION 3.10)

add_subdirectory(internal)

set(LIDAR_PUBLIC_HEADERS
    include/Lidar.h
    include/LidarFeature.h
    include/soslabTypedef.h
)

set(LIDAR_PUBLIC_SOURCES
    src/Lidar.cpp
)

set(LIDAR_INTERNAL_SOURCES
    internal/LidarImpl.cpp
)

set(LIDAR_INTERNAL_HEADERS
    internal/LidarImpl.h
    ${UTILL_INCLUDE_DIR}/GeneralMessage.h
)

source_group("internal/Headers" FILES ${LIDAR_INTERNAL_HEADERS})
source_group("internal/Sources" FILES ${LIDAR_INTERNAL_SOURCES})
source_group("public/Headers" FILES ${LIDAR_PUBLIC_HEADERS})
source_group("public/Sources" FILES ${LIDAR_PUBLIC_SOURCES})

add_library(Lidar SHARED ${LIDAR_PUBLIC_HEADERS} ${LIDAR_PUBLIC_SOURCES} ${LIDAR_INTERNAL_HEADERS} ${LIDAR_INTERNAL_SOURCES})

target_compile_definitions(Lidar PRIVATE SOSLAB_API_EXPORTS)
set_target_properties(Lidar PROPERTIES OUTPUT_NAME ${LIDAR_LIB_NAME})

target_include_directories(Lidar 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/internal
        ${CMAKE_CURRENT_SOURCE_DIR}/internal/FileIO/include
        ${CMAKE_CURRENT_SOURCE_DIR}/internal/Netlink/include
        ${CMAKE_CURRENT_SOURCE_DIR}/internal/Sensor/include
        ${CMAKE_CURRENT_SOURCE_DIR}/internal/LidarRuntime/include
        ${UTILL_INCLUDE_DIR}
        ${ASIO_INCLUDE_DIR}
        ${NLOHMANN_JSON_INCLUDE_DIR}
)
target_link_libraries(Lidar PRIVATE Sensor Netlink FileIO LidarRuntime Win32Lib)

add_custom_command(TARGET Lidar POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_ROOT_DIR}/include"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/include/Lidar.h"
        "${OUTPUT_ROOT_DIR}/include/Lidar.h"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/include/LidarFeature.h"
        "${OUTPUT_ROOT_DIR}/include/LidarFeature.h"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/include/soslabTypedef.h"
        "${OUTPUT_ROOT_DIR}/include/soslabTypedef.h"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${OUTPUT_ROOT_DIR}/include/nlohmann_json"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${NLOHMANN_JSON_INCLUDE_DIR}"
        "${OUTPUT_ROOT_DIR}/include/nlohmann_json"
)