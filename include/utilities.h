/*
 * utilities.h
 *
 *  Created on: Jul 19, 2016
 *      Author: nuc
 */

#ifndef TIME_CALIBRATION_INCLUDE_UTILITIES_H_
#define TIME_CALIBRATION_INCLUDE_UTILITIES_H_

// #includes
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
//OPencv + cv_bridge
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
//Include messages
#include <gnss_node/GnssOut.h>
#include <ublox_node/UbloxRaw.h>
#include <ublox_node/UbloxPVT.h>
#include <ublox_node/GnssRawObs.h>
#include <obs_msgs/GnssRaw.h>


#include <sstream>
#include <vector>

#define FRAME_RATE_COLOR_IMG 15
#define MAX_GPS_TIME_TRACKER 10000
//#defines
//#define _DEBUG
//#define _SAVE_IMAGE

class msg_listener
{
public:
	msg_listener();
	void callbackIRimage(const sensor_msgs::Image::ConstPtr& msg);
	void callbackcolorimage(const sensor_msgs::Image::ConstPtr& msg);
	void callbackubloxpvt (const ublox_node::UbloxPVT::ConstPtr& msg);
	void callbackubloxraw (const ublox_node::UbloxRaw::ConstPtr& msg);
	void callbackgnss (const gnss_node::GnssOut::ConstPtr& msg);
	void gray_2_dec(char s[],int len);
	unsigned long long hex2dec(const std::string &input);
	int GrayToInt(int i);
	void process_capture_delay(double gps_time);
	void conv_rostogps_time();
	std::string cap_gps_time;
	double rep_ros_time, rep_ros_clr_time[FRAME_RATE_COLOR_IMG];
	int color_image_count;
	double gps_sec[MAX_GPS_TIME_TRACKER], gps_rep_ros_sec[MAX_GPS_TIME_TRACKER];
	//std::vector <double> gps_sec;
	//std::vector<double> gps_rep_ros_sec;
	unsigned int gps_sec_disp;
	//std::vector<double>::size_type ubxraw_lr;
	int ubxraw_lr;
	double clock_error;
	std::vector<double_t> clk_err;
#ifdef _SAVE_IMAGE
	int loop_count;
#endif

private:
	//Setup Handle for ROS connection
	ros::NodeHandle nh;
	ros::Subscriber sub_irimg, sub_colorimg, sub_ubxpvt, sub_ubxraw, sub_gnss;
	cv_bridge::CvImagePtr ir_img_ptr, color_img_ptr;
	cv::Mat ir_image, color_img[FRAME_RATE_COLOR_IMG];
};

#endif /* TIME_CALIBRATION_INCLUDE_UTILITIES_H_ */
