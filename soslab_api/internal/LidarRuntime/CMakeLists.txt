cmake_minimum_required(VERSION 3.10)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(HEADER_FILES include/${PROJECT_NAME}.h)
set(SRC_FILES src/${PROJECT_NAME}.cpp)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE Win32Lib)

set_target_properties(${PROJECT_NAME} PROPERTIES 
    OUTPUT_NAME ${RUNTIME_LIB_NAME}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE SOSLAB_API_EXPORTS)

target_include_directories(${PROJECT_NAME} PUBLIC 
    include 
    ${INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../FileIO/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Netlink/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensor/include
    ${ASIO_INCLUDE_DIR}
    ${UTILL_INCLUDE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    Sensor
    Netlink
    FileIO
)