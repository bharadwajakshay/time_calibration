#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
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


/*void callbackIRimage(const sensor_msgs::Image::ConstPtr& msg);
void callbackcolorimage(const sensor_msgs::Image::ConstPtr& msg);
void callbackubloxpvt (const ublox_node::UbloxPVT::ConstPtr& msg);
void callbackubloxraw (const ublox_node::UbloxRaw::ConstPtr& msg);
void callbackgnss (const gnss_node::GnssOut::ConstPtr& msg);*/

class msg_listener
{
public:
	msg_listener();
	void callbackIRimage(const sensor_msgs::Image::ConstPtr& msg);
	void callbackcolorimage(const sensor_msgs::Image::ConstPtr& msg);
	void callbackubloxpvt (const ublox_node::UbloxPVT::ConstPtr& msg);
	void callbackubloxraw (const ublox_node::UbloxRaw::ConstPtr& msg);
	void callbackgnss (const gnss_node::GnssOut::ConstPtr& msg);
	uint16_t vis_gps_time;
private:
	//Setup Handle for ROS connection
	ros::NodeHandle nh;
	ros::Subscriber sub_irimg, sub_colorimg, sub_ubxpvt, sub_ubxraw, sub_gnss;
	cv_bridge::CvImagePtr ir_img_ptr, color_img_ptr;
	cv::Mat ir_image, color_img;
};

msg_listener::msg_listener()
{
	sub_irimg = nh.subscribe("/structure/ir/image",1000,&msg_listener::callbackIRimage,this);
	sub_colorimg = nh.subscribe("/usb_cam/image_raw",1000,&msg_listener::callbackcolorimage,this);
	sub_ubxpvt = nh.subscribe("/ublox/ubxpvt",1000,&msg_listener::callbackubloxpvt,this);
	sub_ubxraw = nh.subscribe("/ublox/ubxraw",1000,&msg_listener::callbackubloxraw,this);
	sub_gnss = nh.subscribe("/gnss/pvt",1000,&msg_listener::callbackubloxpvt,this);
	vis_gps_time = 0;
}

void msg_listener::callbackIRimage(const sensor_msgs::Image::ConstPtr& msg)
{
	//convert sensor msg into opencv image
	ir_img_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);
	ir_img_ptr->image.convertTo(ir_image,CV_8UC1);
	cv::imshow("IR Image",ir_image);
	cv::waitKey(0);
	ROS_INFO_STREAM("Enter the 16-bit value seen in the image");
	std::cin>>vis_gps_time;
	cv::destroyAllWindows();
}

void msg_listener::callbackcolorimage(const sensor_msgs::Image::ConstPtr& msg)
{

}

void msg_listener::callbackubloxpvt(const ublox_node::UbloxPVT::ConstPtr& msg)
{

}

void msg_listener::callbackubloxraw (const ublox_node::UbloxRaw::ConstPtr& msg)
{

}

void msg_listener::callbackgnss (const gnss_node::GnssOut::ConstPtr& msg)
{

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

