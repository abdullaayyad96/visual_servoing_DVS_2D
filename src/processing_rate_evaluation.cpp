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
#include "CornerDetectorARC.h"
#include "CornerDetectorFAST.h"
#include "CornerDetectorHARRIS.h"
#include "CornerDetectorFAHarris.h"
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>



float max_event_generation_rate = 0;
float max_corner_event_generation_rate = 0;
float event_generation_rate = 0;
float corner_event_generation_rate = 0;

float average_event_processing_time = 0;
uint64_t N_events_porcessed = 0;
int cuttof_event_processing = 0;

uint64_t N_events_in_time_window = 0;
uint64_t N_corner_events_in_time_window = 0;
uint64_t window_size = 1e8;
uint64_t last_processed_time = 0;

int queue_size = 100000;
std::vector<dvs_msgs::Event> event_queue;


visual_servoing_davis::CornerDetectorFAHarris corner_detector_;

void davis_feature_callback(const dvs_msgs::EventArray::ConstPtr &msg);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoing");

	ros::NodeHandle nh_;

	ros::Subscriber davis_sub_ = nh_.subscribe("/dvs/events", 1, davis_feature_callback);
	
	ros::spin();

	return 0;
}

void davis_feature_callback(const dvs_msgs::EventArray::ConstPtr &msg)
{
	if 	((msg->events.size() > cuttof_event_processing) && (msg->events.size() < 100000))
	{
		// Packets definition
		dvs_msgs::EventArray packets_corner;
		packets_corner.header = msg->header;
		packets_corner.width = msg->width;
		packets_corner.height = msg->height;

		auto corner_processing_start_time = std::chrono::high_resolution_clock::now();

		// Analysing callback packets
		for (const auto e : msg->events)
		{
			// Separating corner from the detector
			if (corner_detector_.isCorner(e)) // corner
			{
				N_corner_events_in_time_window += 1;
			}
			if (event_queue.size() < queue_size)
			{
				event_queue.push_back(e);
			}
		}

		//event processing rate
		auto corner_processing_end_time = std::chrono::high_resolution_clock::now();		
		auto elapsed_time_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(corner_processing_end_time - corner_processing_start_time);
		
		// float event_processing_rate = msg->events.size() / (elapsed_time_nsecs.count() * 1e-9);
		float event_processing_rate = cuttof_event_processing / (elapsed_time_nsecs.count() * 1e-9);
		average_event_processing_time = (msg->events.size() * event_processing_rate + N_events_porcessed * average_event_processing_time) / (msg->events.size() + N_events_porcessed);
		N_events_porcessed += msg->events.size();

		// ROS_INFO("Event processing speed: %.0f e/s, %0.f nsec/e", event_processing_rate, 1e9/event_processing_rate);
		// ROS_INFO("Average Event processing speed: %.0f e/s, %0.f nsec/e", average_event_processing_time, 1e9/average_event_processing_time);

		if (event_queue.size() >= queue_size)
		{
			corner_processing_start_time = std::chrono::high_resolution_clock::now();
			for (const auto e : event_queue)
			{
				auto output = corner_detector_.isCorner(e);
			}
			corner_processing_end_time = std::chrono::high_resolution_clock::now();
			elapsed_time_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(corner_processing_end_time - corner_processing_start_time);
			event_processing_rate = event_queue.size() / (elapsed_time_nsecs.count() * 1e-9);
			ROS_INFO("Event processing speed: %.0f e/s, %0.f nsec/e", event_processing_rate, 1e9/event_processing_rate);
			event_queue.clear();			
		}




		//event generation rate
		N_events_in_time_window += msg->events.size();
		if (msg->events[msg->events.size()-1].ts.toNSec() > (last_processed_time + window_size))
		{
			event_generation_rate = 1e9 * (float)N_events_in_time_window / (msg->events[msg->events.size()-1].ts.toNSec() - last_processed_time);
			if (event_generation_rate > max_event_generation_rate)
			{
				max_event_generation_rate = event_generation_rate;
			}

			corner_event_generation_rate = 1e9 * (float)N_corner_events_in_time_window / (msg->events[msg->events.size()-1].ts.toNSec() - last_processed_time);
			if (corner_event_generation_rate > max_corner_event_generation_rate)
			{
				max_corner_event_generation_rate = corner_event_generation_rate;
			}

			N_events_in_time_window = 0;
			N_corner_events_in_time_window = 0;
			last_processed_time = msg->events[msg->events.size()-1].ts.toNSec();
			ROS_INFO("Event Genertion rate: %.0f e/s", event_generation_rate);
			ROS_INFO("Max Event Genertion rate: %.0f e/s", max_event_generation_rate);
			ROS_INFO("Corner Event Genertion rate: %.0f e/s", corner_event_generation_rate);
			ROS_INFO("Max Corner Event Genertion rate: %.0f e/s", max_corner_event_generation_rate);
		}
	}
}

