#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
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

//#defines
//#define _DEBUG
#define FRAME_RATE_COLOR_IMG 15
#define MAX_GPS_TIME_TRACKER 10000


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
	std::string cap_gps_time;
	double rep_ros_time, rep_ros_clr_time[FRAME_RATE_COLOR_IMG];
	int color_image_count;
	double gps_sec[MAX_GPS_TIME_TRACKER], gps_rep_sec[MAX_GPS_TIME_TRACKER];
	int ubxraw_lr, gps_time_disp;
#ifdef _DEBUG
	int loop_count;
#endif
private:
	//Setup Handle for ROS connection
	ros::NodeHandle nh;
	ros::Subscriber sub_irimg, sub_colorimg, sub_ubxpvt, sub_ubxraw, sub_gnss;
	cv_bridge::CvImagePtr ir_img_ptr, color_img_ptr;
	cv::Mat ir_image, color_img[FRAME_RATE_COLOR_IMG];
};

msg_listener::msg_listener()
{
	sub_irimg = nh.subscribe("/structure/ir/image",1000,&msg_listener::callbackIRimage,this);
	sub_colorimg = nh.subscribe("/usb_cam/image_raw",1000,&msg_listener::callbackcolorimage,this);
	sub_ubxpvt = nh.subscribe("/ublox/ubxpvt",1000,&msg_listener::callbackubloxpvt,this);
	sub_ubxraw = nh.subscribe("/ublox/ubxraw",1000,&msg_listener::callbackubloxraw,this);
	sub_gnss = nh.subscribe("/gnss/pvt",1000,&msg_listener::callbackubloxpvt,this);
	cap_gps_time = "";
	rep_ros_time = 0;
	memset(rep_ros_clr_time,0,FRAME_RATE_COLOR_IMG);
	color_image_count=0;
	ubxraw_lr =0;
	memset(gps_sec,0,MAX_GPS_TIME_TRACKER);
	memset(gps_rep_sec,0,MAX_GPS_TIME_TRACKER);
	gps_time_disp=0;
#ifdef _DEBUG
	loop_count =0;
#endif
}

void msg_listener::callbackIRimage(const sensor_msgs::Image::ConstPtr& msg)
{
	double min_time_diff = 100000, time_diff = 0;
	char filename_ir[80], filename_clr[80];
	int length;
	int frame_no =0;
	//convert sensor msg into opencv image
	ir_img_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);
	ir_img_ptr->image.convertTo(ir_image,CV_8UC1);
	//capture the reported rostime
	rep_ros_time = msg->header.stamp.toSec();
#ifdef _DEBUG
	ROS_INFO("The reported time :%15.15f",rep_ros_time);
#endif
	//check for the closest color image reported
	for (int i=0;i<color_image_count;i++)
	{
		time_diff = abs(rep_ros_time-rep_ros_clr_time[i]);
		if (time_diff > min_time_diff)
		{
			min_time_diff = time_diff;
			frame_no = i;
		}
	}
	cv::imshow("IR Image",ir_image);
	cv::imshow("Closest Color Image",color_img[frame_no]);
	cv::waitKey(0);
	cv::destroyAllWindows();
#ifdef _DEBUG
	//ROS_INFO("The Entered Byte time :%s",cap_gps_time);
	++loop_count;
	sprintf(filename_ir,"/home/nuc/images/ir_image%d.jpg",loop_count);
	std::cout<<filename_ir<<std::endl;
	cv::imwrite( filename_ir, ir_image );
	sprintf(filename_clr,"/home/nuc/images/color_image%d.jpg",loop_count);
	std::cout<<filename_clr<<std::endl;
	cv::imwrite( filename_clr, color_img[frame_no]);
#endif
	color_image_count = 0;
	ROS_INFO_STREAM("Enter the 16-bit value seen in the image one byte at a time as a Hex value");
	std::cin>>cap_gps_time;
	length=cap_gps_time.length();
	char cap_gps_sec[20];
	std::strcpy(cap_gps_sec,cap_gps_time.c_str());
	gray_2_dec(cap_gps_sec,length);
#ifdef _DEBUG
	std::cout<<"The entered decimal value is"<<gps_time_disp<<std::endl
#endif
}

void msg_listener::callbackcolorimage(const sensor_msgs::Image::ConstPtr& msg)
{
	color_img_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
	rep_ros_clr_time[color_image_count] = msg->header.stamp.toSec();
#ifdef _DEBUG
	ROS_INFO("The reported color image time :%15.15f",rep_ros_clr_time[color_image_count]);
#endif
	color_img_ptr->image.convertTo(color_img[color_image_count],CV_8UC3);
	if (color_image_count >= 12)
		color_image_count=0;
	else
		color_image_count++;
}

void msg_listener::callbackubloxpvt(const ublox_node::UbloxPVT::ConstPtr& msg)
{

}

void msg_listener::callbackubloxraw (const ublox_node::UbloxRaw::ConstPtr& msg)
{
	gps_sec[ubxraw_lr] = msg->gpstime;
	gps_rep_sec[ubxraw_lr]= msg->header.stamp.toSec();
	if (ubxraw_lr >= MAX_GPS_TIME_TRACKER)
		ubxraw_lr =0;
	else
		ubxraw_lr++;

}

void msg_listener::callbackgnss (const gnss_node::GnssOut::ConstPtr& msg)
{

}

void msg_listener::gray_2_dec(char s[],int len)
{
	int decval=0;
	int length = 15;
	for(int i=1;i<=length;i++)
		s[i]=s[i]^s[i-1];
	for (int i=0;i<len;i++)
	{
		decval = decval+(s[i]*(2^(len-i)));
	}
	gps_time_disp = decval;
}


int main(int argc, char **argv)
{
	//Setup local node to listen to messages and initialize ROS
	ros::init(argc, argv,"Time_Calibration");
	//Set loop rate
	//ros::Rate loop_rate(100);
	msg_listener listner;
	ros::spin();
	//loop_rate.sleep();
	return(0);
}

