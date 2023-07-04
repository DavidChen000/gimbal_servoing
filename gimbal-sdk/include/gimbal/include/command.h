#ifndef COMMAND_H
#define COMMAND_H

#include <inttypes.h>

// Size of header and checksums
#define GIMBAL_CMD_NON_PAYLOAD_BYTES 5
// Max. size of a command after packing to bytes
#define GIMBAL_CMD_MAX_BYTES 255
// Max. size of a payload data
#define GIMBAL_CMD_DATA_SIZE                                                   \
  (GIMBAL_CMD_MAX_BYTES - GIMBAL_CMD_NON_PAYLOAD_BYTES)

#define GIMBAL_HEAD 0xAE
#define GIMBAL_VERSION 0x01

// Control gimbal
#define GIMBAL_SPEED_AND_ANGLE_CMD 0x85
#define GIMBAL_SPEED_CTL_MODE 0x01
#define GIMBAL_ANGLE_CTL_MODE 0x02
#define GIMBAL_CTL_HOME 0x03

// Control camera
#define CAMERA_CTL_CMD 0x86
#define CAMERA_START_RECORD_DATA 0x01
#define CAMERA_STOP_RECORD_DATA 0x01
#define CAMERA_TAKE_PHOTO_DATA 0x02

// Get gimbal imu and rotor data
#define GIMBAL_IMU_AND_ROTOR_ANGLE 0x87

#define GIMBAL_SPEED_OFFSET 1500

#endif // COMMAND_H
