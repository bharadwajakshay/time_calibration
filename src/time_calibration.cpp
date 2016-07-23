#include "utilities.h"

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

