#include "gimbal/gimbal.h"
#include "serial/serial.h"
#include <iostream>
#include <string>

#include <getopt.h>
#include <unistd.h>

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
  printf("\nUsage: GetGimbalState [options] <interfaces>\n"
         "\n"
         "Options:\n"
         "-S\tSpecify serial port\n"
         "-b\tSerial baud rate\n"
         "\n"
         "Example:\n"
         "  ./GetGimbalState -S /dev/ttyUSB0 -b 115200\n"
         "\n");

  exit(1);
}

int main(int argc, char *argv[]) {

  serial::Serial ser;
  string serial_port;
  int baud_rate;
  if (argc < 2) {
    usage();
  }

  while (1) {
    int c = getopt(argc, argv, "hS:b:");
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
    default: {
      fprintf(stderr, "Unknown switch %c\n", c);
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

  amov_gimbal_realtime_data rx_data;
  while (1) {

    if (ser.available() > 20) {
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

  ser.close();
  return 0;
}