/*
 * visual_servoing_node.cpp
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */







#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <atomic>
#include <time.h>
#include <sstream>
#include <iostream>
#include <cstdio>
#include "ros/ros.h"
#include <cstdlib>
#include <VisualServoing.h>
//#include "CornerDetectorARC.h"
//#include "CornerDetectorFAST.h"
#include "CornerDetectorHARRIS.h"
#include <slip_detection_davis/object_test.h>




static void Corner_tracking(slip_detection_davis::Visual_Servoing* data_ccollector)
{
	while (ros::ok())
	{
		data_ccollector->corner_detection();
	}   

}


static void robot_EE_movement(slip_detection_davis::Visual_Servoing* data_ccollector)
{
	while (ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::microseconds(10000)); // Sampling

		data_ccollector->ur_manipulation();
	}
}

static void publish_whole_data(slip_detection_davis::Visual_Servoing* data_ccollector)
{
	while (ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::microseconds(10000)); // Sampling

		data_ccollector->publish_data();
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoing");
	ros::NodeHandle nh_;


	slip_detection_davis::Visual_Servoing* processs;
	processs = new slip_detection_davis::CornerDetector_HARRIS;


	// Threads

	//std::thread th1(&publish_frame_millisec_desired, processs);
	std::thread th1(&Corner_tracking, processs);
	std::thread th3(&robot_EE_movement, processs); // only when controller used
	std::thread th4(&publish_whole_data, processs); // only when controller used


	ros::spin();

	delete processs;

	return 0;
}

