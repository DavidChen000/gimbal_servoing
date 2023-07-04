#include "gimbal.h"

void gimbal_cmd_control_pack(amov_gimbal_control &p, SerialCommand &cmd) {
  cmd.init(GIMBAL_HEAD);
  cmd.version = GIMBAL_VERSION;
  cmd.cmd = GIMBAL_SPEED_AND_ANGLE_CMD;
  if (GIMBAL_SPEED_CTL_MODE == p.mode) {
    // speed control
    cmd.writeByte(GIMBAL_SPEED_CTL_MODE);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(p.speedROLL);
    cmd.writeWord(p.speedPITCH);
    cmd.writeWord(p.speedYAW);
  } else if (GIMBAL_ANGLE_CTL_MODE == p.mode) {
    // angle control
    cmd.writeByte(GIMBAL_ANGLE_CTL_MODE);
    cmd.writeWord(p.angleROLL);
    cmd.writeWord(p.anglePITCH);
    cmd.writeWord(p.angleYAW);
    cmd.writeWord(p.speedROLL);
    cmd.writeWord(p.speedPITCH);
    cmd.writeWord(p.speedYAW);
  } else if (GIMBAL_CTL_HOME == p.mode) {
    // home_pos
    cmd.writeByte(GIMBAL_CTL_HOME);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(0);
    cmd.writeWord(0);
  }
}

void camera_cmd_control_pack(amov_camera_control &p, SerialCommand &cmd) {
  cmd.init(GIMBAL_HEAD);
  cmd.version = GIMBAL_VERSION;
  cmd.cmd = CAMERA_CTL_CMD;
  cmd.writeByte(p.action);
}

uint8_t gimbal_cmd_realtime_data_unpack(amov_gimbal_realtime_data &p,
                                        SerialCommand &cmd) {
  if (cmd.id == GIMBAL_HEAD && cmd.cmd == GIMBAL_IMU_AND_ROTOR_ANGLE) {
    cmd.readWordArr(p.imu_angle, 3);
    cmd.readWordArr(p.rotor_angle, 3);
  }
  if (cmd.checkLimit()) {
    return 0;
  } else {
    return PARSER_ERROR_WRONG_DATA_SIZE;
  }
}
