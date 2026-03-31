
set(NETLINK_LIB_NAME "Netlink${ARCH_SUFFIX}$<$<CONFIG:Debug>:_debug>$<$<CONFIG:Release>:_release>")
set(FILEIO_LIB_NAME "FileIO${ARCH_SUFFIX}$<$<CONFIG:Debug>:_debug>$<$<CONFIG:Release>:_release>")
set(SENSOR_LIB_NAME "Sensor${ARCH_SUFFIX}$<$<CONFIG:Debug>:_debug>$<$<CONFIG:Release>:_release>")
set(RUNTIME_LIB_NAME "LidarRuntime${ARCH_SUFFIX}$<$<CONFIG:Debug>:_debug>$<$<CONFIG:Release>:_release>")

add_subdirectory(FileIO)
add_subdirectory(Netlink)
add_subdirectory(Sensor)
add_subdirectory(LidarRuntime)
