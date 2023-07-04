
#include <getopt.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include "gimbal/gimbal.h"
#include "serial/serial.h"

const int multiple = 100;

using namespace std;

class MainComObj : public GIMBAL_ComObj {
  serial::Serial *ser;

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

void usage(void) {
  printf("\nUsage: GimbalAngleControl [options] <interfaces>\n"
         "\n"
         "Options:\n"
         "-S\tSpecify serial port\n"
         "-b\tSerial baud rate\n"
         "-r\tRoll angle control\n"
         "-p\tPitch angle control\n"
         "-y\tYaw angle control\n"
         "-s\tAngle rate\n"
         "\n"
         "Example:\n"
         " ./GimbalAngleControl -S /dev/ttyUSB0 -b 115200 -r 10 -p 10 -y "
         "10 -s 10"
         "\n");
  exit(1);
}

int main(int argc, char *argv[]) {
  serial::Serial ser;
  string serial_port;
  int baud_rate;
  float roll;
  float pitch;
  float yaw;
  float angle_rate;

  if (argc < 2) {

    usage();
  }

  while (1) {
    int c = getopt(argc, argv, "hS:b:r:p:y:s:");
    if (c == -1) {
      break;
    }

    switch (c) {
    case 'h': {
      usage();
      break;
    }
    case 'S': {
      serial_port = optarg;
      cout << "serial_port " << serial_port << endl;
      break;
    }
    case 'b': {
      baud_rate = atoi(optarg);
      cout << "baud_rate " << baud_rate << endl;
      break;
    }
    case 'r': {
      roll = atof(optarg);
      cout << "roll " << roll << endl;
      break;
    }
    case 'p': {
      pitch = atof(optarg);
      cout << "pitch " << pitch << endl;
      break;
    }
    case 'y': {
      yaw = atof(optarg);
      cout << "yaw " << yaw << endl;
      break;
    }
    case 's': {
      angle_rate = atof(optarg);
      cout << "angle_rate " << angle_rate << endl;
      break;
    }
    default: {
      fprintf(stderr, "unknown switch %c\n", c);
      usage();

      break;
    }
    }
  }

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

  MainComObj maincom;
  maincom.init(&ser);
  GIMBAL_Parser gimbal_parser;
  gimbal_parser.init(&maincom);

  amov_gimbal_control gimbal_control = {0, 0, 0, 0, 0, 0, 0};
  gimbal_control.mode = GIMBAL_ANGLE_CTL_MODE;
  gimbal_control.speedROLL = angle_rate * multiple;
  gimbal_control.speedPITCH = angle_rate * multiple;
  gimbal_control.speedYAW = angle_rate * multiple;
  gimbal_control.angleROLL = roll * multiple;
  gimbal_control.anglePITCH = pitch * multiple;
  gimbal_control.angleYAW = yaw * multiple;

  gimbal_cmd_control(gimbal_control, gimbal_parser);
  ser.close();
  return 0;
}