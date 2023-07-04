#ifndef GIMBAL_CMD_H
#define GIMBAL_CMD_H

#include "gimbal_parser.h"

// Gimbal control
typedef struct {
  uint8_t mode;
  int16_t angleROLL;
  int16_t anglePITCH;
  int16_t angleYAW;
  int16_t speedROLL;
  int16_t speedPITCH;
  int16_t speedYAW;

} amov_gimbal_control;

// Camera control
typedef struct {
  uint8_t action;
} amov_camera_control;

// Get gimabl realtime data
typedef struct {
  int16_t imu_angle[3];   // relative angle of imu
  int16_t rotor_angle[3]; // relative angle of each rotor
} amov_gimbal_realtime_data;

void gimbal_cmd_control_pack(amov_gimbal_control &p, SerialCommand &cmd);
inline uint8_t gimbal_cmd_control(amov_gimbal_control &p,
                                  GIMBAL_Parser &parser) {
  SerialCommand cmd;
  gimbal_cmd_control_pack(p, cmd);
  return parser.send_cmd(cmd);
}

void camera_cmd_control_pack(amov_camera_control &p, SerialCommand &cmd);

inline uint8_t camera_cmd_control(amov_camera_control &p,
                                  GIMBAL_Parser &parser) {
  SerialCommand cmd;
  camera_cmd_control_pack(p, cmd);
  return parser.send_cmd(cmd);
}

uint8_t gimbal_cmd_realtime_data_unpack(amov_gimbal_realtime_data &p,
                                        SerialCommand &cmd);

#endif // GIMBAL_CMD_H
