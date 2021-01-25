/*
 * VisualServoing.cpp
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */


//TODO: change to cfg parameters
#define myVSmode 1
#define updateTracking 0
#define publish_corners 0
#define evaluate_corner_processing_time 0
#define evaluate_corner_detection_time 0
//false for frame, true for dvs


#include "VisualServoing.h"
namespace visual_servoing_davis
{
Visual_Servoing::Visual_Servoing() {

	// Subscribers
	tracking_mode = pnh_.subscribe("/tracking_mode", 0, &Visual_Servoing::tracking_mode_callback, this);
	detection_mode = pnh_.subscribe("/detection_mode", 0, &Visual_Servoing::detection_mode_callback, this);
	cam_info_subs_ = pnh_.subscribe("dvs/camera_info", 10, &Visual_Servoing::CamInfoCallback, this);
	if (myVSmode)
	{
		davis_sub_ = pnh_.subscribe("/dvs/events", 2, &Visual_Servoing::davis_feature_callback, this);
	}
	else
	{
		frame_image_sub = pnh_.subscribe("/dvs/image_raw", 2, &Visual_Servoing::frame_image_callback, this);
	}

	//Pubishers
	centroid_pub = pnh_.advertise<dvs_msgs::Event>("/object_center", 1);
	processed_corner_pub = pnh_.advertise<dvs_msgs::EventArray>("/object_corners", 1);
	corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events_soft", 1);
	pub_heatmap =pnh_.advertise<sensor_msgs::Image>("/corner_heatmap", 1);
	pub_corners_image =pnh_.advertise<sensor_msgs::Image>("/corners", 1);
	event_frames_pub =pnh_.advertise<sensor_msgs::Image>("/event_frame", 1);
	cmd_vel_pub = pnh_.advertise<geometry_msgs::Twist>("/ur_cmd_vel", 2);
	cmd_rotate_ee_pub = pnh_.advertise<std_msgs::Float64>("/ur_rotate_ee", 1);
	cmd_rotate_ee_pub_x = pnh_.advertise<std_msgs::Float64>("/ur_rotate_ee_x", 1);
	cmd_mode_pub = pnh_.advertise<std_msgs::Bool>("/ur_detection_mode", 1);

	this->f_ = boost::bind(&Visual_Servoing::parameter_callback, this, _1, _2);
	this->server_.setCallback(this->f_);

	sae= Eigen::MatrixXd::Zero(sensor_width_,sensor_height_);
}


void Visual_Servoing::CamInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
	sensor_msgs::CameraInfo cam_info_ = *camera_info;
	this->cam_.fromCameraInfo(cam_info_);
	this->cam_initialized = true;

	//Disable camera info callbacks
	this->cam_info_subs_.shutdown();        
}

void Visual_Servoing::parameter_callback(visual_servoing_davis::VSCfgConfig &config, uint32_t level)
{
	this->corner_detector_.update_params(config.eHarris_queue_size, config.eHarris_window_size, config.eHarris_threshold);
	this->harris_th = config.Harris_threshold;
	this->block_size_ = config.Harris_block_size;
	this->aperture_size_ = config.Harris_aperture_size;
	this->velocity = config.target_velocity;
	this->center_offset_threshold = config.center_offset_threshold;
	this->time_decay_factor = config.heatmap_time_decay_factor;
	this->corner_alpha = config.heatmap_corner_weight;
	this->heatmap_thresh = config.heatmap_threshold;
	this->dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(config.heatmap_dilate_kernel_size, config.heatmap_dilate_kernel_size));
	this->corner_var_constant = config.tracking_corner_var;
	this->new_event_var = config.tracking_event_var;
	this->likelihood_thresh = config.tracking_likelihood_thresh;
	this->k_p = config.tracking_kp;
	this->k_d = config.tracking_kd;
	this->tracking_err_thresh = config.tracking_err_thresh;
	this->false_tracking_thresh = config.false_tracking_thresh;
}

void Visual_Servoing::davis_feature_callback(const dvs_msgs::EventArray::ConstPtr &msg)
{
	// Packets definition
	dvs_msgs::EventArray packets_corner;
	packets_corner.header = msg->header;
	packets_corner.width = msg->width;
	packets_corner.height = msg->height;

	// Create a frame from events
	this->davis_frame = cv::Mat::zeros(this->sensor_height_, this->sensor_width_, CV_8UC3); 							// frame was declared in the header file

	auto corner_processing_start_time = std::chrono::high_resolution_clock::now();

	// Analysing callback packets
	for (const auto e : msg->events)
	{		
		// Separating corner from the detector
		if (this->corner_detector_.isCorner(e)) // corner
		{
			C_e=e;
			// if (this->cam_initialized)
			// {
			// 	this->rectified_point = this->cam_.rectifyPoint(cv::Point2d(e.x, e.y));
			// 	if ((this->rectified_point.x < this->sensor_width_-1) and (this->rectified_point.x > 0) and 
			// 		(this->rectified_point.y < this->sensor_height_-1) and (this->rectified_point.y > 0))
			// 	{
			// 		C_e.x = this->rectified_point.x;
			// 		C_e.y = this->rectified_point.y;
			// 	}
			// }	
			corner_queue.push(C_e);
			if (publish_corners)
			{
				packets_corner.events.push_back(e);
			}
		}

	 	// this->davis_frame.at<cv::Vec3b>(e.y, e.x) = cv::Vec3b(255, 255, 255);								// Assigning white color @ element (e.y,e.x)
	}
	
	if (evaluate_corner_processing_time)
	{
		auto corner_processing_end_time = std::chrono::high_resolution_clock::now();		
      	auto elapsed_time_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(corner_processing_end_time - corner_processing_start_time);
		float event_processing_rate = msg->events.size() / (elapsed_time_nsecs.count() * 1e-9);
		ROS_INFO("Event processing speed: %.0f e/s, %0.f nsec/e", event_processing_rate, 1e9/event_processing_rate);
	}

	if (publish_corners)
	{
		corner_events_pub.publish(packets_corner);
		packets_corner.events.clear();
	}


// 	if (this->cam_initialized)
// 	{
// 		this->cam_.rectifyImage(this->davis_frame, this->davis_frame);
// 	}	
// 	//cvtColor(this->davis_frame, this->davis_frame_mono, cv::COLOR_BGR2GRAY);
// 	//this->contourDetection(this->davis_frame_mono);
// 	sensor_msgs::Image temp_ros_image;
// 	this->createROSFrame(this->davis_frame, temp_ros_image);
// 	event_frames_pub.publish(temp_ros_image);
}

void Visual_Servoing::createROSFrame(cv::Mat input_frame, sensor_msgs::Image &ros_image)
{
	std_msgs::Header header; // empty header
	header.seq = 1; // user defined counter
	header.stamp = ros::Time::now(); // time

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, input_frame);

	img_bridge.toImageMsg(ros_image); // from cv_bridge to sensor_msgs::Image
}


void Visual_Servoing::frame_image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
	this->img_bridge = *cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	this->frame = this->img_bridge.image;
	this->harrisCornerDetection(this->frame, msg->header.stamp);
}

void Visual_Servoing::harrisCornerDetection(cv::Mat input_frame, ros::Time current_ts)
{
	cv::cornerHarris(input_frame, this->harris_frame_, this->block_size_, this->aperture_size_, this->k_);
	//cv::normalize(this->harris_frame_, this->harris_frame_, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	
	dvs_msgs::EventArray packets_corner;
	packets_corner.width = this->sensor_width_;
	packets_corner.height = this->sensor_height_;

	int num_corners = 0;

	for( int i = 0; i < this->harris_frame_.rows ; i++ )
    {
        for( int j = 0; j < this->harris_frame_.cols; j++ )
        {
            if( (int) this->harris_frame_.at<float>(i,j) > this->harris_th )
            {
				C_e.x = j;
				C_e.y = i;
				C_e.ts = current_ts;
				corner_queue.push(C_e);
				packets_corner.events.push_back(C_e);
				num_corners++;
            }
        }
    }
	corner_events_pub.publish(packets_corner);
	packets_corner.events.clear();

	//NOTE: start commenting for dynamic tracking
	if(num_corners==0 && (this->current_mode == robot_mode::tracking) && !myVSmode)
	{
		this->no_corner_switch_counter++;
		if (this->no_corner_switch_counter >= this->no_corner_switch_thresh)
		{
			this->no_corner_switch_counter = 0;
			std::cout << "no track" << std::endl;
			this->cmd_vel_twist.linear.x = 0;
			this->cmd_vel_twist.linear.y = 0;	
			cmd_vel_pub.publish(this->cmd_vel_twist);
			this->current_mode = robot_mode::detection;
			this->detection_span = 2;
			this->false_tracking_counter = 0;
			ROS_INFO("Switch back to detection mode, %lld corners, %lld", (long long)this->corners.total(), (long long)ros::Time::now().toNSec());	
		}
	}
	//NOTE: start commenting for dynamic tracking
}


void Visual_Servoing::contourDetection(cv::Mat input_frame)
{
	cv::Mat contour_dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
	cv::dilate(input_frame, input_frame, contour_dilate_kernel);
	cv::findContours(input_frame, this->contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

	//find largest contour
	int largest_contour_size = 0;
	int largest_ctr_idx = 0;
	for (int i=0; i<this->contours.size(); i++)
	{
		if(this->contours[i].size() > largest_contour_size)
		{
			this->largest_contour = this->contours[i];
			largest_contour_size = this->contours[i].size();
			largest_ctr_idx = i;
		}
	}

	if (largest_contour_size>50)
	{
		this->largest_rect = cv::boundingRect(this->largest_contour);
		this->contour_center.x = this->largest_rect.x+this->largest_rect.width/2;
		this->contour_center.y = this->largest_rect.y+this->largest_rect.height/2;	
	}

	cv::drawContours(this->davis_frame, this->contours, largest_ctr_idx, cv::Scalar(255,0,0));
	this->davis_frame.at<cv::Vec3b>(this->contour_center.y, this->contour_center.x) = cv::Vec3b(0, 255, 0);
}

void Visual_Servoing::detection_mode_callback(const std_msgs::Bool &msg)
{
	std_msgs::Bool ur10_detection_mode;
	ur10_detection_mode.data = msg.data;
	cmd_mode_pub.publish(ur10_detection_mode);

	//read depth
	boost::shared_ptr<geometry_msgs::PoseStamped const> cam_pose;
	cam_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/dvs/pose");
	this->depth = cam_pose->pose.position.z - this->item_height;

	//Set UR reference frame to davis frame
	URX::desiredTCP srv;
	srv.request.frame = "davis";
	ros::service::call("set_TCP", srv);

	this->detection_span = 1;
	if (msg.data)
	{
		this->current_mode = this->detection;
		this->first_detection_time = ros::Time::now().toSec();
		for(int i=0 ; i < this->corners.total() ; i++)
		{
			this->corners_var[i] = this->corner_var_constant;
		}
		ROS_INFO("Start detection mode: %lld", (long long)ros::Time::now().toNSec());
	}
	else
	{
		this->current_mode = this->idle;
		this->cmd_vel_twist.linear.x = 0;
		this->cmd_vel_twist.linear.y = 0;	
	  	cmd_vel_pub.publish(this->cmd_vel_twist);
	}	
	//this->random_initial_center.x = (int)(rand() % this->sensor_width_);
	//this->random_initial_center.y = (int)(rand() % this->sensor_height_);
	this->random_initial_center.x = (int)(this->sensor_width_/2);
	this->random_initial_center.y = 0;//(int)(this->sensor_width_);
	this->corner_heatmap_cv = cv::Mat(180, 240, CV_64FC1);
}

void Visual_Servoing::tracking_mode_callback(const std_msgs::Bool &msg)
{
	if (msg.data)
	{
		this->current_mode = this->tracking;
		for(int i=0 ; i < this->corners.total() ; i++)
		{
			this->corners_var[i] = this->corner_var_constant;
		}
	}
	else
	{
		this->current_mode = this->idle;
		this->cmd_vel_twist.linear.x = 0;
		this->cmd_vel_twist.linear.y = 0;	
	  	cmd_vel_pub.publish(this->cmd_vel_twist);
	}	
	std_msgs::Bool ur10_detection_mode;
	ur10_detection_mode.data = false;
	cmd_mode_pub.publish(ur10_detection_mode);
}

void Visual_Servoing::corner_detection()
{
	if (this->corner_queue.size() > 0)
	{
		auto corner_detection_start_time = std::chrono::high_resolution_clock::now();
		this->temp_e = this->corner_queue.front();
		this->corner_queue.pop();
		// this->corner_heatmap_time_update(ros::Time::now().toNSec()/1000000000);

		if (this->current_mode==robot_mode::detection || this->current_mode==robot_mode::idle || this->current_mode==robot_mode::rotate)
		{
			this->heatmap_corner_queue.push(this->temp_e);

			if (this->heatmap_corner_queue.size() > this->heatmap_queue_size)
			{
				while(this->heatmap_corner_queue.size() > 0)
				{

					this->corner_heatmap_add_event(this->heatmap_corner_queue.front().y, 10, 15, this->heatmap_corner_queue.front().x, 10, 15);
					this->corner_heatmap_time_update(this->heatmap_corner_queue.front().ts.toNSec()/1000000000);
					this->heatmap_corner_queue.pop();
				}
				

				cv::dilate(this->corner_heatmap_cv,  this->dilated_heatmap, this->dilate_kernel);

				cv::threshold(this->dilated_heatmap, this->upper_thresh_map, this->heatmap_thresh, 10.0, CV_THRESH_TOZERO);
				cv::threshold(this->dilated_heatmap, this->lower_thresh_map, this->heatmap_thresh, 0.1, CV_THRESH_BINARY_INV);
				this->dilated_heatmap = this->upper_thresh_map + this->lower_thresh_map;
				
				this->processed_heatmap = (this->dilated_heatmap == this->corner_heatmap_cv);

				cv::findNonZero(this->processed_heatmap, this->corners);

				//average corners for center
				object_center.x = 0;
				object_center.y = 0;
				corners_var.clear();
				for (int i = 0 ; i < this->corners.total() ; i++)
				{
					object_center.x += this->corners.at<cv::Point>(i).x / this->corners.total();
					object_center.y += this->corners.at<cv::Point>(i).y / this->corners.total();
					corners_var.push_back( 1 / this->corner_heatmap_cv.at<double>(this->corners.at<cv::Point>(i).y, this->corners.at<cv::Point>(i).x));
				}
				
				// if( (this->current_mode==robot_mode::detection) && (ros::Time::now().toSec() - this->first_detection_time) > this->detection_span)
				// {
				// 	std_msgs::Bool ur10_tracking_mode;
				// 	ur10_tracking_mode.data = true;				
				// 	this->tracking_mode_callback(ur10_tracking_mode);
					
				// 	ROS_INFO("Switch to tracking mode, %lld corners, %lld time", (long long)this->corners.total(), (long long)ros::Time::now().toNSec());
				// }
				this->last_detection_time = ros::Time::now().toSec();
			}
		}

		else if (this->current_mode==robot_mode::tracking)
		{
			this->update_corner_variance(this->temp_e.ts.toSec()); 
			this->update_corners(this->temp_e);
			//average corners for center
			object_center.x = 0;
			object_center.y = 0;

			for (int i = 0 ; i < this->corners.total() ; i++)
			{
				object_center.x += this->corners.at<cv::Point>(i).x / this->corners.total();
				object_center.y += this->corners.at<cv::Point>(i).y / this->corners.total();
			}

			if (updateTracking)
			{
				this->corner_heatmap_add_event(this->temp_e.y, 10, 15, this->temp_e.x, 10, 15);
				this->corner_heatmap_time_update(this->temp_e.ts.toNSec()/1000000000);

				if ((ros::Time::now().toSec() - this->last_detection_time) > this->detection_correction_priod)
				{
					this->last_detection_time = ros::Time::now().toSec();

					cv::dilate(this->corner_heatmap_cv,  this->dilated_heatmap, this->dilate_kernel);

					cv::threshold(this->dilated_heatmap, this->upper_thresh_map, this->heatmap_thresh, 10.0, CV_THRESH_TOZERO);
					cv::threshold(this->dilated_heatmap, this->lower_thresh_map, this->heatmap_thresh, 0.1, CV_THRESH_BINARY_INV);
					this->dilated_heatmap = this->upper_thresh_map + this->lower_thresh_map;
					
					this->processed_heatmap = (this->dilated_heatmap == this->corner_heatmap_cv);

					cv::Mat temp_corners;
					cv::findNonZero(this->processed_heatmap, temp_corners);

					//average corners for center
					double temp_object_center_x = 0;
					double temp_object_center_y = 0;
					for (int i = 0 ; i < temp_corners.total() ; i++)
					{
						temp_object_center_x += temp_corners.at<cv::Point>(i).x / temp_corners.total();
						temp_object_center_y += temp_corners.at<cv::Point>(i).y / temp_corners.total();
					}

					double temp_tracking_err =std::sqrt(std::pow(this->object_center.x - temp_object_center_x, 2) + std::pow(this->object_center.y - temp_object_center_y, 2));

					if (temp_tracking_err > this->tracking_err_thresh)
					{
						this->false_tracking_counter++;
						ROS_INFO("False tracking counter: %lld", this->false_tracking_counter);
					}
					else
					{
						this->false_tracking_counter = 0;
					}
					

					if (this->false_tracking_counter >= this->false_tracking_thresh)
					{
						this->cmd_vel_twist.linear.x = 0;
						this->cmd_vel_twist.linear.y = 0;	
						cmd_vel_pub.publish(this->cmd_vel_twist);
						this->current_mode = robot_mode::detection;
						this->detection_span = 2;
						this->false_tracking_counter = 0;
						ROS_INFO("Switch back to detection mode, %lld corners, %f error distance, %lld", (long long)this->corners.total(), (float)temp_tracking_err, (long long)ros::Time::now().toNSec());
					}		
				}
			}
			//NOTE: end commenting for dynamic tracking			
		}
		this->last_event_time = this->temp_e.ts.toNSec()/1000000000;
		// this->last_event_time =  ros::Time::now().toNSec()/1000000000;


		if (evaluate_corner_detection_time)
		{
			auto corner_detection_end_time = std::chrono::high_resolution_clock::now();
      		auto elapsed_time_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(corner_detection_end_time - corner_detection_start_time);

			if (this->current_mode==robot_mode::tracking)
			{
				this->tracking_procerssing_time = (this->tracking_processing_N * this->tracking_procerssing_time + elapsed_time_nsecs.count() * 1e-9) / (this->tracking_processing_N + 1);
				this->tracking_processing_N++;
				if (this->tracking_processing_N % this->heatmap_queue_size == 0)
				{
					this->tracking_processing_N = 0;		
					ROS_INFO("Event tracking corner processing time speed: %.0f nsecs", this->tracking_procerssing_time * 1e9);
				}
			}
			else
			{
				this->detection_procerssing_time = (this->detection_processing_N * this->detection_procerssing_time + elapsed_time_nsecs.count() * 1e-9) / (this->detection_processing_N + 1);
				this->detection_processing_N++;
				if (this->detection_processing_N % this->heatmap_queue_size == 0)
				{
					this->detection_processing_N = 0;
					ROS_INFO("Event detection corner processing time speed: %.0f nsecs", this->detection_procerssing_time * 1e9);
				}
			}
			
		}		
	}	
}

void Visual_Servoing::ee_orientation()
{

	//Detects two furthest corners and calculate angle

	//   this->corner_a_y = 0;
	//   this->corner_a_x = 0;
	//   this->distance_benchmark = 0;
	//   double corner_a_x_benchmark;
	//   for(int i = 0; i < this->corners.total() ; i++)
	//   {
	// 	  corner_a_x_benchmark = this->corners.at<cv::Point>(i).x;
	// 	  if (this->corner_a_x<= corner_a_x_benchmark)
	// 	  {
	// 		 this->corner_a_x = corners.at<cv::Point>(i).x;
	// 		 this->corner_a_y =  corners.at<cv::Point>(i).y;
	// 		 this->corner_a_idx = i;
	// 	  }
	//   }
	
	//   for(int i = 0; i < this->corners.total() ; i++)
	//   {
	//   	  this->distance = std::sqrt(std::pow(this->corner_a_x - this->corners.at<cv::Point>(i).x, 2) + std::pow(this->corner_a_y - this->corners.at<cv::Point>(i).y, 2));
	// 	  if(this->distance >= this->distance_benchmark)
	// 	  {
	// 		this->distance_benchmark = this->distance;
	// 		this->corner_b_x = this->corners.at<cv::Point>(i).x;
	// 		this->corner_b_y = this->corners.at<cv::Point>(i).y;
	// 		this->corner_b_idx = i;
	// 	  }
	//   }

	//   // orientation angle (diagonal) in radians
	//   double temp_angle = std::atan2(this->corner_b_y - this->corner_a_y, this->corner_b_x - this->corner_a_x);
	
	this->corner_a_y = 0;
	this->corner_a_x = 0;
	this->distance_benchmark = 0;
	for(int i = 0; i < this->corners.total() ; i++)
	{
		this->distance = std::sqrt(std::pow(this->object_center.x - this->corners.at<cv::Point>(i).x, 2) + std::pow(this->object_center.y - this->corners.at<cv::Point>(i).y, 2));
		if(this->distance >= this->distance_benchmark)
		{
			this->distance_benchmark = this->distance;
			this->corner_a_x = this->corners.at<cv::Point>(i).x; //corner_a is the furthest corner from the centroid
			this->corner_a_y = this->corners.at<cv::Point>(i).y;
			this->corner_a_idx = i;
		}
	}

	// orientation angle (diagonal) in radians
	double temp_angle = std::atan2(this->object_center.y - this->corner_a_y, this->object_center.x - this->corner_a_x);
	
	//take shortest rotation
	while (temp_angle >= M_PI/2)
	{
		temp_angle = temp_angle - M_PI;
	}
	while (temp_angle <= -M_PI/2)
	{
		temp_angle = temp_angle + M_PI;
	}
	
	this->orientation_angle.data = temp_angle;


		/*if(this->actual_distance > 20) // 20 is measured in centimeters
		{
			//Use two vaccum grippers after orienting to the orientation_angle and moving to the centroid
		
		} elseif(this->actual_distance < 20) // 20 is measured in centimeters
		{
			//use one vaccum gripper
			//no need for orientation angle
		}*/

}
void Visual_Servoing::ur_manipulation()
{
	if (this->current_mode==robot_mode::idle)
	{
		//TODO
	}
	if (this->current_mode==robot_mode::detection)
	{
		// double distance = std::sqrt(std::pow(this->random_initial_center.x - (((double)this->sensor_width_)/2.0),2) + std::pow(this->random_initial_center.y - (((double)this->sensor_height_)/2.0),2));
		// if (distance >= this->center_offset_threshold)
		// {
		// 	this->cmd_vel_twist.linear.x = (this->random_initial_center.x - (((double)this->sensor_width_)/2.0)) * 0.05 / distance;
		// 	this->cmd_vel_twist.linear.y = (this->random_initial_center.y - (((double)this->sensor_height_)/2.0)) * 0.05 / distance;
		// }
		// else
		// {
		// 	this->current_mode=robot_mode::rotate;
		// 	this->cmd_vel_twist.linear.x = 0;
		// 	this->cmd_vel_twist.linear.y = 0;
		// }		
		// cmd_vel_pub.publish(this->cmd_vel_twist);
		if (myVSmode)
		{
			this->orientation_angle.data = 0.1;
			cmd_rotate_ee_pub_x.publish(this->orientation_angle);
			std::this_thread::sleep_for(std::chrono::seconds(2));
			this->orientation_angle.data = -0.1;
			cmd_rotate_ee_pub_x.publish(this->orientation_angle);
			std::this_thread::sleep_for(std::chrono::seconds(2));
		}
		else
		{
			std::this_thread::sleep_for(std::chrono::seconds(3));
		}
		std_msgs::Bool ur10_tracking_mode;
		ur10_tracking_mode.data = true;				
		ROS_INFO("Switch to tracking mode, %lld corners, %lld time", (long long)this->corners.total(), (long long)ros::Time::now().toNSec());
		this->tracking_mode_callback(ur10_tracking_mode);
	}
	else if (this->current_mode==robot_mode::tracking)
	{
		double distance = std::sqrt(std::pow(this->object_center.x - (((double)this->sensor_width_)/2.0),2) + std::pow(this->object_center.y - (((double)this->sensor_height_)/2.0),2));
		this->ref_pixel_vel = this->k_p * distance;
		double target_velocity = this->ref_pixel_vel * this->depth / this->cam_.fx();
		if (target_velocity > this->velocity)
		{
			target_velocity = this->velocity;
		}
		if (distance >= 8 * this->center_offset_threshold)
		{
			this->cmd_vel_twist.linear.x = (this->object_center.x - (((double)this->sensor_width_)/2.0)) * target_velocity / distance;
			this->cmd_vel_twist.linear.y = (this->object_center.y - (((double)this->sensor_height_)/2.0)) * target_velocity / distance;
			this->tracking_zero_counter = 0;
		}
		else if (distance >= 5 * this->center_offset_threshold)
		{
			this->cmd_vel_twist.linear.x = (this->object_center.x - (((double)this->sensor_width_)/2.0)) * target_velocity / (2*distance);
			this->cmd_vel_twist.linear.y = (this->object_center.y - (((double)this->sensor_height_)/2.0)) * target_velocity / (2*distance);
			this->tracking_zero_counter = 0;
		}
		else if (distance >= this->center_offset_threshold)
		{
			this->cmd_vel_twist.linear.x = (this->object_center.x - (((double)this->sensor_width_)/2.0)) * target_velocity / (6*distance);
			this->cmd_vel_twist.linear.y = (this->object_center.y - (((double)this->sensor_height_)/2.0)) * target_velocity / (6*distance);
			this->tracking_zero_counter = 0;
		}
		else
		{
			this->cmd_vel_twist.linear.x = 0;
			this->cmd_vel_twist.linear.y = 0;
			this->tracking_zero_counter++;
		}
		//NOTE: start commenting for dynamic tracking
		if(this->tracking_zero_counter > 50)
		{
			this->current_mode=robot_mode::rotate;
			ROS_INFO("Switch to ee rotate mode:, %lld", (long long)ros::Time::now().toNSec());
		}		
		//NOTE: end commenting for dynamic tracking
		cmd_vel_pub.publish(this->cmd_vel_twist);
	}
	else if (this->current_mode==robot_mode::rotate)
	{
		this->ee_orientation();
		cmd_rotate_ee_pub.publish(this->orientation_angle);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		this->current_mode = robot_mode::pickup;
		ROS_INFO("Switch to grasp mode:, %lld", (long long)ros::Time::now().toNSec());
	}	  
	else if (this->current_mode==robot_mode::pickup)
	{
		if (!this->pickup_status)
		{
			ros::service::call("ur_pickup", this->empty_service);
			this->pickup_status = true;
			ROS_INFO("Grasp complete:, %lld", (long long)ros::Time::now().toNSec());
		}
		//this->current_mode = robot_mode::idle;
	}
}

void Visual_Servoing::publish_data()
{
	this->corner_heatmap_cv_mono8 = cv::Scalar(0);

	//add lines
	//cv::line(this->corner_heatmap_cv_mono8, cv::Point(this->sensor_width_/2, 0), cv::Point(this->sensor_width_/2, this->sensor_height_), cv::Scalar(255));
	//cv::line(this->corner_heatmap_cv_mono8, cv::Point(0, this->sensor_height_/2), cv::Point(this->sensor_width_, this->sensor_height_/2), cv::Scalar(255));

	//iterate through corners and set points
	for (int i = 0 ; i < this->corners.total() ; i++)
	{
		this->corner_heatmap_cv_mono8.at<uint8_t>(this->corners.at<cv::Point>(i)) = 255;
	}
	this->corner_heatmap_cv_mono8.at<uint8_t>(this->object_center.y, this->object_center.x) = 255;

	cv::Mat colored_SACE_SAVE = cv::Mat(180, 240, CV_8UC3);

	cv::cvtColor(this->corner_heatmap_cv_mono8, colored_SACE_SAVE, cv::COLOR_GRAY2RGB);
	cv::Vec3b color = cv::Vec3b(135, 206, 255);
	colored_SACE_SAVE.at<cv::Vec3b>(this->object_center.y, this->object_center.x) = color;
	// colored_SACE_SAVE.at<cv::Vec3b>(this->contour_center.y, this->contour_center.x) = color;

	std_msgs::Header header; // empty header
	header.seq = 1; // user defined counter
	header.stamp = ros::Time::now(); // time
	//this->img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, this->corner_heatmap_cv_mono8);
	this->img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, colored_SACE_SAVE);

	this->img_bridge.toImageMsg(this->corner_heatmap_image); // from cv_bridge to sensor_msgs::Image

	pub_corners_image.publish(this->corner_heatmap_image); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

	cv::convertScaleAbs(this->corner_heatmap_cv, this->corner_heatmap_cv_mono8, 255, 0.0);

	this->img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, this->corner_heatmap_cv_mono8);

	this->img_bridge.toImageMsg(this->corner_heatmap_image); // from cv_bridge to sensor_msgs::Image

	pub_heatmap.publish(this->corner_heatmap_image); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

	dvs_msgs::Event center_event;
	center_event.x = object_center.x;
	center_event.y = object_center.y;
	this->centroid_pub.publish(center_event);

	dvs_msgs::EventArray packets_corner;
	packets_corner.width = this->sensor_width_;
	packets_corner.height = this->sensor_height_;
	//iterate through corners and set points
	for (int i = 0 ; i < this->corners.total() ; i++)
	{
		center_event.x = this->corners.at<cv::Point>(i).x;
		center_event.y = this->corners.at<cv::Point>(i).y;
		packets_corner.events.push_back(center_event);
	}
	this->processed_corner_pub.publish(packets_corner);
	packets_corner.events.clear();

}


void Visual_Servoing::corner_heatmap_add_event(int event_x, double x_var, int x_window, int event_y, double y_var, int y_window)
{
	int x_start_index = std::max(event_x-x_window, 0);
	int x_end_index = std::min(event_x+x_window, this->sensor_height_-1);
	int y_start_index = std::max(event_y-y_window, 0);
	int y_end_index = std::min(event_y+y_window, this->sensor_width_-1);

	for (int i = x_start_index ; i <= x_end_index ; i++)
	{
		for (int j = y_start_index ; j <= y_end_index ; j++)
		{
			this->corner_heatmap_cv.at<double>(i,j) += this->corner_alpha * std::exp(-0.5 * ((std::pow(i-event_x, 2)/x_var) + (std::pow(j-event_y, 2)/y_var)));// / (2*M_PI*std::sqrt(x_var*y_var));
		}
	}

	//scaledown to avoid flip overs
	double min_val, max_val;

	cv::minMaxLoc(this->corner_heatmap_cv, &min_val, &max_val);
	if(max_val > (2.8))
	{
			this->corner_heatmap_cv =  this->corner_heatmap_cv * (2.8) / max_val;
	}
	else if(std::abs(min_val) == std::numeric_limits<double>::infinity())
	{
		this->corner_heatmap_cv = cv::Mat::zeros(this->sensor_height_, this->sensor_width_, CV_64FC1);
	}
}

void Visual_Servoing::corner_heatmap_time_update(double new_timestamp)
{
	this->corner_heatmap_cv = exp(-this->time_decay_factor * (new_timestamp - this->last_event_time)) * this->corner_heatmap_cv;
}

void Visual_Servoing::update_corner_variance(double new_timestamp)
{
	//TODO: update in better way
	//set corner variances to constant
	for (int i=0 ; i<this->corners_var.size() ; i++)
	{
		this->corners_var[i] = this->corner_var_constant;
		//this->corners_var[i] += this->timestamp_update_var * (new_timestamp - this->last_event_time);
		if (this->corners_var[i] > this->corner_var_constant)
		{
			this->corners_var[i] = this->corner_var_constant;
		}
	}
}

double Visual_Servoing::calculate_gaussian_likelihood(double distance, double variance)
{
	return std::exp(-0.5*std::pow(distance, 2)/(variance));
}

int Visual_Servoing::find_corner_association(dvs_msgs::Event new_event)
{
	double distance = 0;
	double min_distance = 1000;
	int min_distance_idx = 0;
	for (int i=0 ; i < this->corners.total() ; i++)
	{
		distance = std::sqrt( std::pow(this->corners.at<cv::Point>(i).x - new_event.x, 2) + std::pow(this->corners.at<cv::Point>(i).y -new_event.y, 2));

		if (distance < min_distance)
		{
			min_distance = distance;
			min_distance_idx = i;
		}
	}
		
	if (this->calculate_gaussian_likelihood(min_distance, this->corners_var[min_distance_idx]) > this->likelihood_thresh)
	{
		return min_distance_idx;
	}	

	return -1;
}

void Visual_Servoing::update_corners(dvs_msgs::Event new_event)
{
	int association = this->find_corner_association(new_event);
	if (association!=-1)
	{
		//update other corners with same movement but with higher variance
		//NOTE: start commenting for dynamic tracking
		for (int i=0 ; i<this->corners.total() ; i++)
		{
			if (i!=association)
			{
			this->corners.at<cv::Point>(i).x  = ( (this->new_event_var + this->corners_var[i]) * this->corners.at<cv::Point>(i).x + this->corners_var[i] * (new_event.x - this->corners.at<cv::Point>(association).x)) / (this->corners_var[i] + this->new_event_var);
			this->corners.at<cv::Point>(i).y  = ( (this->new_event_var + this->corners_var[i]) * this->corners.at<cv::Point>(i).y + this->corners_var[i] * (new_event.y - this->corners.at<cv::Point>(association).y)) / (this->corners_var[i] + this->new_event_var);
			this->corners_var[i] = (this->corners_var[i] * this->new_event_var) / (this->corners_var[association] + this->new_event_var);
			}
		}
		//NOTE: end commenting for dynamic tracking
		this->corners.at<cv::Point>(association).x  = (this->new_event_var * this->corners.at<cv::Point>(association).x + this->corners_var[association] * new_event.x) / (this->corners_var[association] + this->new_event_var);
		this->corners.at<cv::Point>(association).y  = (this->new_event_var * this->corners.at<cv::Point>(association).y + this->corners_var[association] * new_event.y) / (this->corners_var[association] + this->new_event_var);
		this->corners_var[association] = (this->corners_var[association] * this->new_event_var) / (this->corners_var[association] + this->new_event_var);	  
	}
}

Visual_Servoing::~Visual_Servoing() {
	// TODO Auto-generated destructor stub
}

}
