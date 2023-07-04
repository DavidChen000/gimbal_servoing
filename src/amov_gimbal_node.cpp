#include "amov_gimbal_sdk_ros/amov_gimbal_node.h"
#include "amov_gimbal_sdk_ros/GimbalControl.h"
#include "amov_gimbal_sdk_ros/GimbalState.h"
#include <iostream>
#include <ros/ros.h>
#include <thread>

using namespace std;

amov_gimbal_sdk_ros::GimbalControl gimbal_control_;
bool gimbal_control_flag = false;

amov_gimbal_sdk_ros::GimbalState gimbal_state_;
class MainComObj : public GIMBAL_ComObj {
  serial::Serial *ser = NULL;

public:
  void init(serial::Serial *s) { ser = s; }
  virtual uint16_t getBytesAvailable() { return ser->available(); }
  virtual uint8_t readByte() {
    uint8_t a_char = 0;
    ser->read(&a_char, 1);
    return a_char;
  }
  virtual void writeByte(uint8_t b) { ser->write(&b, 1); }
  virtual uint16_t getOutEmptySpace() {
    printf("getOutEmptySpace");
    return 0xFFFF;
  }
};

class AmovGimbalROS {

private:
  serial::Serial *ser = NULL;
  MainComObj maincom;
  GIMBAL_Parser gimbal_parser;
  bool gimbal_get_state = true;

public:
  amov_gimbal_control gimbal_control = {0, 0, 0, 0, 0, 0, 0};
  amov_camera_control cam_control = {0};
  amov_gimbal_realtime_data rx_data = {0, 0, 0, 0, 0, 0};

public:
  AmovGimbalROS(serial::Serial *s);
  ~AmovGimbalROS();
  void gimbal_angle_and_rate_control(void);
  void gimbal_angle_rate_control(void);
  void gimbal_go_home(void);
  void get_gimbal_state(void);
  void set_camera_take_photo(void);
  void set_camera_record(void);
};

AmovGimbalROS::AmovGimbalROS(serial::Serial *s) {
  ser = s;
  maincom.init(ser);
  gimbal_parser.init(&maincom);
  std::thread t(&AmovGimbalROS::get_gimbal_state, this);
  t.detach();
}
AmovGimbalROS::~AmovGimbalROS(void) {
  this->gimbal_get_state = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ser->close();
}

void AmovGimbalROS::gimbal_angle_and_rate_control(void) {
  gimbal_control.mode = GIMBAL_ANGLE_CTL_MODE;

  gimbal_cmd_control(gimbal_control, gimbal_parser);
}
void AmovGimbalROS::gimbal_angle_rate_control(void) {
  gimbal_control.mode = GIMBAL_SPEED_CTL_MODE;

  gimbal_cmd_control(gimbal_control, gimbal_parser);
}

void AmovGimbalROS::gimbal_go_home(void) {
  gimbal_control.mode = GIMBAL_CTL_HOME;
  gimbal_cmd_control(gimbal_control, gimbal_parser);
}
void AmovGimbalROS::get_gimbal_state(void) {

  while (this->gimbal_get_state) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (ser->available() > 20) {
      if (gimbal_parser.read_cmd() == true) {
        gimbal_cmd_realtime_data_unpack(rx_data, gimbal_parser.in_cmd);
        cout << "=================AMOVLAB GIMBAL G1 STATE================="
             << endl;
        cout << "IMU roll angle: " << rx_data.imu_angle[0] * 0.01 << "°"
             << endl;
        cout << "IMU pitch angle: " << rx_data.imu_angle[1] * 0.01 << "°"
             << endl;
        cout << "IMU yaw angle: " << rx_data.imu_angle[2] * 0.01 << "°" << endl;
        cout << "Rotor roll angle: " << rx_data.rotor_angle[0] * 0.01 << "°"
             << endl;
        cout << "Rotor pitch angle: " << rx_data.rotor_angle[1] * 0.01 << "°"
             << endl;
        cout << "Rotor yaw angle: " << rx_data.rotor_angle[2] * 0.01 << "°"
             << endl;
      }
    }
  }
}
void AmovGimbalROS::set_camera_take_photo(void) {
  cam_control.action = CAMERA_TAKE_PHOTO_DATA;
  camera_cmd_control(cam_control, gimbal_parser);
}
void AmovGimbalROS::set_camera_record(void) {
  cam_control.action = CAMERA_START_RECORD_DATA;
  camera_cmd_control(cam_control, gimbal_parser);
}

void control_callback(const amov_gimbal_sdk_ros::GimbalControl::ConstPtr &msg) {
  gimbal_control_ = *msg;
  gimbal_control_flag = true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amov_gimbal_sdk_ros");
  ros::NodeHandle nh("~");

  serial::Serial ser;
  string serial_port = "/dev/ttyUSB0";
  int baud_rate = 115200;

  nh.getParam("serial_port", serial_port);
  nh.getParam("baud_rate", baud_rate);

  ros::Publisher gimbal_state_pub =
      nh.advertise<amov_gimbal_sdk_ros::GimbalState>(
          "/amov_gimbal_ros/gimbal_state", 1000);
  ros::Subscriber sub = nh.subscribe<amov_gimbal_sdk_ros::GimbalControl>(
      "/amov_gimbal_ros/gimbal_control", 1000, control_callback);

  try {
    ser.setPort(serial_port);
    ser.setBaudrate(baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(500);
    ser.setTimeout(to);
    ser.open();

  } catch (serial::IOException &e) {
    std::cout << "Unable open serial" << std::endl;
    return -1;
  }

  if (ser.isOpen()) {
    AmovGimbalROS *gimbal = new AmovGimbalROS(&ser);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      if (gimbal_control_flag == true) {
        gimbal->gimbal_control.angleROLL = gimbal_control_.roll_angle * 100;
        gimbal->gimbal_control.anglePITCH = gimbal_control_.pitch_angle * 100;
        gimbal->gimbal_control.angleYAW = gimbal_control_.yaw_angle * 100;
        gimbal->gimbal_control.speedROLL = gimbal_control_.roll_rate * 100;
        gimbal->gimbal_control.speedPITCH = gimbal_control_.pitch_rate * 100;
        gimbal->gimbal_control.speedYAW = gimbal_control_.yaw_rate * 100;

        if (gimbal_control_.mode == GIMBAL_SPEED_CTL_MODE) {
          gimbal->gimbal_angle_rate_control();
        } else if (gimbal_control_.mode == GIMBAL_ANGLE_CTL_MODE) {
          gimbal->gimbal_angle_and_rate_control();
        } else if (gimbal_control_.mode == GIMBAL_CTL_HOME) {
          gimbal->gimbal_go_home();
        }
        gimbal_control_flag = false;
      }

      gimbal_state_.imu_angle[0] = gimbal->rx_data.imu_angle[0] * 0.01;
      gimbal_state_.imu_angle[1] = gimbal->rx_data.imu_angle[1] * 0.01;
      gimbal_state_.imu_angle[2] = gimbal->rx_data.imu_angle[2] * 0.01;
      gimbal_state_.rotor_angle[0] = gimbal->rx_data.rotor_angle[0] * 0.01;
      gimbal_state_.rotor_angle[1] = gimbal->rx_data.rotor_angle[1] * 0.01;
      gimbal_state_.rotor_angle[2] = gimbal->rx_data.rotor_angle[2] * 0.01;

      gimbal_state_pub.publish(gimbal_state_);
      loop_rate.sleep();
    }
    delete gimbal;
  }

  return 0;
}