/*
 * VisualServoing.h
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_
#define SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_



#pragma once

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
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
//#include <dvs_msgs/Event.h>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "timer.h"
#include <tuple>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <sys/time.h>
#include <slip_detection_davis/object_test.h>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <ros/console.h>
// Gripper
//#include <electric_gripper_baxter.h>


#include <stddef.h>
#include <stdlib.h>

#define M_PI 3.14159265358979323846



namespace slip_detection_davis
{
class Visual_Servoing {
public:
	Visual_Servoing();
	virtual ~Visual_Servoing();


	virtual  double isCorner_Edge_HARRISi (const dvs_msgs::Event &e)= 0;
//	   virtual  bool isCornerFAST (const dvs_msgs::Event &e)= 0;
//	   virtual  bool isCornerARC (const dvs_msgs::Event &e)= 0;



// callback and functions
	void Davis_feature_Callback(const dvs_msgs::EventArray::ConstPtr &msg);
	void tracking_mode_callback(const std_msgs::Bool &msg);
	void detection_mode_callback(const std_msgs::Bool &msg);
	void corner_detection();
	void ee_orientation();
	void ur_manipulation();
	void publish_data();

	void update_corner_variance(double new_timestamp);
	void update_corners(dvs_msgs::Event corner_event); 
	
	double calculate_gaussian_likelihood(double distance, double variance);

	int find_corner_association(dvs_msgs::Event new_event);

	void corner_heatmap_add_event(int event_x, double x_var, int x_window, int event_y, double y_var, int y_window);
	void corner_heatmap_time_update(double new_timestamp);

	// ROS service
	bool ServiceCallback1(object_test::Request  &req,object_test::Response &res);
	bool ServiceCallback2(object_test::Request  &req,object_test::Response &res);

	double th1= -0.001;//-0.0000001,
	double th2= 4;//8; //3;//7 //8
	int e_max, c_max;

	enum robot_mode {idle, detection, tracking, rotate, pickup};
private:
	Eigen::MatrixXd sae;
	static const int sensor_width_ = 240;
	static const int sensor_height_ = 180;
	// Tuning parameters
  	uint64_t  timeSinceEpochMillisec();
	typedef std::chrono::high_resolution_clock h_clock;
	typedef std::chrono::duration<float, std::milli> duration;
  	h_clock::time_point  start = h_clock::now();
	utils::time::Timer<std::chrono::nanoseconds> timero;

  	h_clock::time_point  t_grasp_start = h_clock::now();
  	h_clock::time_point  t_slip_start = h_clock::now();
	  
  	double elaspedTimeMs=0;
  	double elaspedTimeMst;
  	double elaspedTimeGrip=0;

  	geometry_msgs::WrenchStamped FT_Biased;

  	dvs_msgs::Event C_e, E_e, C_c, temp_e;

	std::queue<dvs_msgs::Event> corner_queue; 


	robot_mode current_mode = idle; 
	bool pickup_status = false; //false: job not yet done, true job done

	//parameters for corner detection mode
	double last_event_time = 0;
	double first_detection_time = 0;
	double last_detection_time = 0;
	double detection_span = 3;
	double detection_correction_priod = 0.3;
	double tracking_err_thresh = 10;
	int false_tracking_counter = 0;
	int false_tracking_thresh = 3;
	double time_decay_factor = 0.1;
	double corner_alpha = 0.15;

	cv::Mat corner_heatmap_cv = cv::Mat(180, 240, CV_64FC1);
    cv::Mat dilated_heatmap = cv::Mat(180, 240, CV_8UC1);
	cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat processed_heatmap = cv::Mat::zeros(180, 240, CV_8UC1);
	
	double heatmap_thresh = 0.6;
    cv::Mat upper_thresh_map = cv::Mat(180, 240, CV_8UC1);
    cv::Mat lower_thresh_map = cv::Mat(180, 240, CV_8UC1);

	//for visualization with ROS
    cv::Mat corner_heatmap_cv_mono8 = cv::Mat(180, 240, CV_8UC1);
	sensor_msgs::Image corner_heatmap_image, SAVE_image;
	cv_bridge::CvImage img_bridge;

	//parameters for corner tracking
	cv::Mat corners;
	std::vector<double> corners_var;
	double corner_var_constant = 400;
	double timestamp_update_var = 2 * std::pow(10, 4);
	double new_event_var = 3000; //1400
	double likelihood_thresh = 0.8;//0.8
	int corner_association;

	cv::Point object_center;
	cv::Point random_initial_center;

	//parameters for robot rotation
	int corner_a_x, corner_a_y, corner_b_x, corner_b_y, corner_a_y_benchmark;
	double distance;
	double distance_benchmark;
	int corner_a_idx, corner_b_idx;
	

	//variables for UR manipulation
	double velocity = 0.06;
	double center_offset_threshold = 2;
	geometry_msgs::Twist cmd_vel_twist;
	std_msgs::Float64 orientation_angle;
	
  	// Ros service server
	ros::ServiceServer goal_service, sample_service, exp_reset_service;

	//Subscriber and publisher and nodehandle
	ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub,complete_data, centroid_pub, pub_heatmap, pub_corners_image, cmd_vel_pub, cmd_rotate_ee_pub, cmd_mode_pub; // Publish classified online events
	ros::Subscriber davis_sub_; // Subscribe data from Davis
	ros::Subscriber tracking_mode, detection_mode; // Subscribe data from Davis
	ros::NodeHandle pnh_;
	// time and header
	ros::Time st;
	std_msgs::Header head_data;
	
	std_srvs::Empty empty_service;


    // Counters
	int count_raw, count_flat, count_corner, count_edge, t_max_data_compare;

    //defined parameters
    int  frame_ms =1;
    int  threshhold_edge=200;
  	std::string condition ="10_HF_10_MF_10_V";
  	std::string object ="Metal";
  	std::string detector ="Harris";

  	int elaspedTime_grasp ;
  	int elaspedTime_slip ;

  	int control_edge_raw_max, control_corner_raw_max;
};
}
#endif /* SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_ */
