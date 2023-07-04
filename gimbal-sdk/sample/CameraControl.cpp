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
  printf("\nUsage: CameraControl [options] <interfaces>\n"
         "\n"
         "Options:\n"
         "-S\tSpecify serial port\n"
         "-b\tSerial baud rate\n"
         "-a\tCamera action 1:take a photo 2:record and stop record\n"
         "\n"
         "Example:\n"
         " ./CameraControl -S /dev/ttyUSB0 -b 115200 -a 1"
         "\n");
  exit(1);
}

int main(int argc, char *argv[]) {
  serial::Serial ser;
  string serial_port;
  int baud_rate;
  int camera_cation = 0;
  amov_camera_control cam_control = {0};
  if (argc < 2) {
    usage();
  }

  while (1) {
    int c = getopt(argc, argv, "hS:b:a:");
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
      cout << "Serial_port " << serial_port << endl;
      break;
    }
    case 'b': {
      baud_rate = atoi(optarg);
      cout << "Baud_rate " << baud_rate << endl;
      break;
    }
    case 'a': {
      camera_cation = atoi(optarg);
      cout << "action " << camera_cation << endl;
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
  if (camera_cation == 1) {
    cam_control.action = CAMERA_TAKE_PHOTO_DATA;
    camera_cmd_control(cam_control, gimbal_parser);
  } else if (camera_cation == 2) {
    cam_control.action = CAMERA_START_RECORD_DATA;
    camera_cmd_control(cam_control, gimbal_parser);
  }

  ser.close();
  return 0;
}