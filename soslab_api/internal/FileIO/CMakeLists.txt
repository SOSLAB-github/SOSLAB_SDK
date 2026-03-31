cmake_minimum_required(VERSION 3.10)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

set(HEADER_FILES include/${PROJECT_NAME}.h)
set(SRC_FILES src/${PROJECT_NAME}.cpp)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SRC_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES 
    OUTPUT_NAME ${FILEIO_LIB_NAME}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE SOSLAB_API_EXPORTS)

target_include_directories(${PROJECT_NAME} PUBLIC 
    include
    ${INCLUDE_DIR}
    ${UTILL_INCLUDE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)