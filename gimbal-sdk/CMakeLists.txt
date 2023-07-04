cmake_minimum_required(VERSION 2.8.3)
project(amovgimbalsdk)


## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
   )


list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)


## Add library
add_library(serial ${serial_SRCS})
target_link_libraries(serial rt pthread)
add_library(crc32  include/gimbal/crc32.cpp)
add_library(gimbal_cmd  include/gimbal/gimbal_cmd.cpp)


## Include headers
include_directories(include)


add_executable(GetGimbalState sample/GetGimbalState.cpp)
target_link_libraries(GetGimbalState serial crc32 gimbal_cmd)

add_executable(GimbalAngleRateControl sample/GimbalAngleRateControl.cpp)
target_link_libraries(GimbalAngleRateControl serial crc32 gimbal_cmd)


add_executable(GimbalAngleControl sample/GimbalAngleControl.cpp)
target_link_libraries(GimbalAngleControl serial crc32 gimbal_cmd)


add_executable(CameraControl sample/CameraControl.cpp)
target_link_libraries(CameraControl serial crc32 gimbal_cmd)