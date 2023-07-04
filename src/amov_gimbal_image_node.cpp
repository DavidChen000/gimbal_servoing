#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

using namespace std;

image_transport::Publisher image_pub;
sensor_msgs::ImagePtr image_msg;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "amov_gimbal_image_node");
  ros::NodeHandle nh("~");

  string camera_image_topic = "/amov_gimbal_ros/gimbal_image";
  string camera_ip = "192.168.2.64";
  int camera_image_fps = 30;
  int camera_height = 720;
  int camera_width = 1280;
  int camera_bitrate = 4000000;
  nh.getParam("camera_image_topic", camera_image_topic);
  nh.getParam("camera_ip", camera_ip);
  nh.getParam("camera_image_fps", camera_image_fps);
  nh.getParam("camera_height", camera_height);
  nh.getParam("camera_width", camera_width);
  nh.getParam("camera_bitrate", camera_bitrate);

  std::ostringstream oss;
  oss << "rtspsrc location=rtsp://" << camera_ip
      << ":554/H264?W=" << camera_width << "&H=" << camera_height
      << "&FPS=" << camera_image_fps << "&BR=" << camera_bitrate
      << " latency=10 caps='application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=\
                (string)H264,width=1280,height=720,framerate=30/1' !\
                rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! \
                video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! \
                videoconvert ! appsink sync=false";

  std::string pipline_str = oss.str();

  image_transport::ImageTransport it(nh);
  image_pub = it.advertise(camera_image_topic, 1);
  cv::VideoCapture capture;
  cv::Mat frame;
  capture.open(pipline_str);
  if (!capture.isOpened()) {
    ROS_FATAL("Camera failed to open!");
    return -1;
  } else {
    ROS_INFO("Successfully opened the camera!");
  }
  ros::Rate loop_rate(30);

  while (ros::ok()) {
    capture.read(frame);
    if (!frame.empty()) {
      image_msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      image_pub.publish(image_msg);
      // cv::imshow("web_camera_image", frame);
      cv::waitKey(20);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  capture.release();
  return 0;
}