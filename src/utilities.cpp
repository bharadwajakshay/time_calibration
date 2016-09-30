/*
 * utilities.cpp
 *
 *  Created on: Jul 19, 2016
 *      Author: nuc
 */
#include "utilities.h"


msg_listener::msg_listener()
{
	sub_irimg = nh.subscribe("/structure/ir/image",1000,&msg_listener::callbackIRimage,this);
#ifdef _COLOR_IMAGE
	sub_colorimg = nh.subscribe("/usb_cam/image_raw",1000,&msg_listener::callbackcolorimage,this);
#endif
	sub_ubxpvt = nh.subscribe("/ublox/ubxpvt",1000,&msg_listener::callbackubloxpvt,this);
	sub_ubxraw = nh.subscribe("/ublox/ubxraw",1000,&msg_listener::callbackubloxraw,this);
	sub_gnss = nh.subscribe("/gnss/pvt",1000,&msg_listener::callbackubloxpvt,this);
	cap_gps_time = "";
	rep_ros_time = 0;
	memset(rep_ros_clr_time,0,FRAME_RATE_COLOR_IMG);
	color_image_count=0;
	ubxraw_lr =0;
	memset(gps_sec,0,MAX_GPS_TIME_TRACKER);
	memset(gps_rep_ros_sec,0,MAX_GPS_TIME_TRACKER);
	//gps_sec.resize(MAX_GPS_TIME_TRACKER);
	//gps_rep_ros_sec.resize(MAX_GPS_TIME_TRACKER);
	//gps_sec.clear();
	//gps_rep_ros_sec.clear();
	gps_sec_disp=0;
	std::setprecision(10);
	clock_error=0;
#ifdef _SAVE_IMAGE
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

#ifdef _COLOR_IMAGE
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
#endif
	cv::imshow("IR Image",ir_image);
#ifdef _COLOR_IMAGE
	cv::imshow("Closest Color Image",color_img[frame_no]);
#endif
	cv::waitKey(0);
	cv::destroyAllWindows();
#ifdef _SAVE_IMAGE
	//ROS_INFO("The Entered Byte time :%s",cap_gps_time);
	++loop_count;
	sprintf(filename_ir,"/home/nuc/images/ir_image%d.jpg",loop_count);
	std::cout<<filename_ir<<std::endl;
	cv::imwrite( filename_ir, ir_image );
#ifdef _COLOR_IMAGE
	sprintf(filename_clr,"/home/nuc/images/color_image%d.jpg",loop_count);
	std::cout<<filename_clr<<std::endl;
	cv::imwrite( filename_clr, color_img[frame_no]);
#endif
#endif
	color_image_count = 0;
	ROS_INFO_STREAM("Enter the 16-bit value seen in the image one byte at a time as a Hex value");
	std::cin>>cap_gps_time;
	gps_sec_disp=GrayToInt(hex2dec(cap_gps_time));

#ifdef _DEBUG
	std::cout<<"The entered decimal value is"<<gps_sec_disp<<std::endl
#endif

	conv_rostogps_time();

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
#ifdef _DEBUG
	ROS_INFO_STREAM("Entered UBLOX RAW message call back");
#endif
	gps_sec[ubxraw_lr]=msg->gpstime;
	gps_rep_ros_sec[ubxraw_lr]=msg->header.stamp.toSec();
	//gps_sec.pop_back(); //remove the additional insertion
	if (ubxraw_lr >= MAX_GPS_TIME_TRACKER)
		ubxraw_lr =0;
	else
		ubxraw_lr++;
}

void msg_listener::callbackgnss (const gnss_node::GnssOut::ConstPtr& msg)
{

}

/***********************************************************************
 * //From http://www.zedwood.com/article/cpp-hexdec-conversion-function
 **********************************************************************/
unsigned long long int msg_listener::hex2dec(const std::string &input)
{
    unsigned long long n;
    std::stringstream ss;
    ss << std::hex << std::uppercase << input;
    ss >> n;
    return n;
}

/***********************************************************************
 *From http://www.richelbilderbeek.nl/CppGrayToInt.htm
 *From Modified from Press et al., 2002, Numerical Recipies in C++,
 *From ISBN 0 521 75033 4
 **********************************************************************/
int msg_listener::GrayToInt(int i)
{
  int power = 1;
  while (1)
  {
    const int j = i >> power;
    i ^= j;
    if (j == 0 || power == 16) return i;
    power <<= 1;
  }
}


void msg_listener::conv_rostogps_time()
{
	double closest_time;
	int cons_ele=0;
	double min=100000;
	for (int i=0;i<MAX_GPS_TIME_TRACKER;i++)
	{
		double diff=gps_rep_ros_sec[i]-rep_ros_time;
		if (fabs(diff) < min)
		{
			min = fabs(gps_rep_ros_sec[i]-rep_ros_time);
			cons_ele=i;
		}
	}
	double corrsponding_gps_sec = gps_sec[cons_ele];
	double diff_in_ros_time = rep_ros_time - gps_rep_ros_sec[cons_ele];
	double est_gps_time_cap = corrsponding_gps_sec + diff_in_ros_time;
#ifdef _DEBUG
	std::cout<<"Current GPS time is"<<corrsponding_gps_sec<<std::endl;
	std::cout<<"estimated GPS time is"<<est_gps_time_cap<<std::endl;
#endif

	process_capture_delay(est_gps_time_cap);
}

void msg_listener::process_capture_delay(double gps_time)
{
	//based on MAtlab Code to convert Displayed GPStime to Error
	double mod_gps_sec = fmod(gps_time, 64);
	double mod_gps_time = gps_sec_disp / 1024;
	clock_error = mod_gps_time - mod_gps_sec;
	ROS_INFO("Calculated Clock Error is %f",clock_error);
	clk_err.push_back(clock_error);
}
