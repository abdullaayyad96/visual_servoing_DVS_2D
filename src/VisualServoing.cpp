/*
 * VisualServoing.cpp
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */

#include "VisualServoing.h"
namespace visual_servoing_davis
{
Visual_Servoing::Visual_Servoing() {

	// Subscribers
	davis_sub_ = pnh_.subscribe("/dvs/events", 0, &Visual_Servoing::davis_feature_callback, this);
	tracking_mode = pnh_.subscribe("/tracking_mode", 0, &Visual_Servoing::tracking_mode_callback, this);
	detection_mode = pnh_.subscribe("/detection_mode", 0, &Visual_Servoing::detection_mode_callback, this);
	cam_info_subs_ = pnh_.subscribe("dvs/camera_info", 10, &Visual_Servoing::CamInfoCallback, this);
	//frame_image_sub = pnh_.subscribe("/dvs/image_raw", 0, &Visual_Servoing::frame_image_callback, this);

	//Pubishers
	centroid_pub = pnh_.advertise<dvs_msgs::EventArray>("/object_center", 1);
	corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events_soft", 1);
	pub_heatmap =pnh_.advertise<sensor_msgs::Image>("/corner_heatmap", 1);
	pub_corners_image =pnh_.advertise<sensor_msgs::Image>("/corners", 1);
	event_frames_pub =pnh_.advertise<sensor_msgs::Image>("/event_frame", 1);
	cmd_vel_pub = pnh_.advertise<geometry_msgs::Twist>("/ur_cmd_vel", 1);
	cmd_rotate_ee_pub = pnh_.advertise<std_msgs::Float64>("/ur_rotate_ee", 1);
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
	this->time_decay_factor = config.heatmap_time_decay_factor;
	this->corner_alpha = config.heatmap_corner_weight;
	this->heatmap_thresh = config.heatmap_threshold;
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
	
	// Analysing callback packets
	for (const auto e : msg->events)
	{		
		// Separating corner from the detector
		if (this->corner_detector_.isCorner(e)) // corner
		{
			C_e=e;
			if (this->cam_initialized)
			{
				this->rectified_point = this->cam_.rectifyPoint(cv::Point2d(e.x, e.y));
				if ((this->rectified_point.x < this->sensor_width_-1) and (this->rectified_point.x > 0) and 
					(this->rectified_point.y < this->sensor_height_-1) and (this->rectified_point.y > 0))
				{
					C_e.x = this->rectified_point.x;
					C_e.y = this->rectified_point.y;
				}
			}	
			corner_queue.push(C_e);
			packets_corner.events.push_back(e);
		}

	 	this->davis_frame.at<cv::Vec3b>(e.y, e.x) = cv::Vec3b(255, 255, 255);								// Assigning white color @ element (e.y,e.x)
	}

	corner_events_pub.publish(packets_corner);
	packets_corner.events.clear();


	if (this->cam_initialized)
	{
		this->cam_.rectifyImage(this->davis_frame, this->davis_frame);
	}	
	sensor_msgs::Image temp_ros_image;
	this->createROSFrame(this->davis_frame, temp_ros_image);
	event_frames_pub.publish(temp_ros_image);
	cvtColor(this->davis_frame, this->davis_frame_mono, cv::COLOR_BGR2GRAY);
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

	this->harrisCornerDetection(this->frame);
}

void Visual_Servoing::harrisCornerDetection(cv::Mat input_frame)
{
	cv::cornerHarris(input_frame, this->harris_frame_, this->block_size_, this->aperture_size_, this->k_);
	//cv::normalize(this->harris_frame_, this->harris_frame_, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
	
	for( int i = 0; i < this->harris_frame_.rows ; i++ )
    {
        for( int j = 0; j < this->harris_frame_.cols; j++ )
        {
            if( (int) this->harris_frame_.at<float>(i,j) > this->harris_th )
            {
				C_e.x = j;
				C_e.y = i;
				corner_queue.push(C_e);
            }
        }
    }
}


void Visual_Servoing::detection_mode_callback(const std_msgs::Bool &msg)
{
	std_msgs::Bool ur10_detection_mode;
	ur10_detection_mode.data = msg.data;
	cmd_mode_pub.publish(ur10_detection_mode);
	this->detection_span = 6;
	if (msg.data)
	{
		this->current_mode = this->detection;
		this->first_detection_time = ros::Time::now().toSec();
		for(int i=0 ; i < this->corners.total() ; i++)
			this->corners_var[i] = this->corner_var_constant;
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
	this->random_initial_center.y = 0;
}

void Visual_Servoing::tracking_mode_callback(const std_msgs::Bool &msg)
{
	if (msg.data)
	{
		this->current_mode = this->tracking;
		for(int i=0 ; i < this->corners.total() ; i++)
			this->corners_var[i] = this->corner_var_constant;
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
		this->temp_e = this->corner_queue.front();
		this->corner_queue.pop();
		this->corner_heatmap_add_event(this->temp_e.y, 10, 15, this->temp_e.x, 10, 15);
		this->corner_heatmap_time_update(this->temp_e.ts.toNSec()/1000000000);

		if (this->current_mode==robot_mode::detection || this->current_mode==robot_mode::idle || this->current_mode==robot_mode::rotate)
		{
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
			
			if( (this->current_mode==robot_mode::detection) && (ros::Time::now().toSec() - this->first_detection_time) > this->detection_span)
			{
				std_msgs::Bool ur10_tracking_mode;
				ur10_tracking_mode.data = true;				
				this->tracking_mode_callback(ur10_tracking_mode);

				ROS_DEBUG("Switch to tracking, %f corners", this->corners.total());
			}
			this->last_detection_time = ros::Time::now().toSec();
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
					std::cout << this->false_tracking_counter << std::endl;
				}
				else
				{
					this->false_tracking_counter = 0;
				}
				

				if (this->false_tracking_counter >= this->false_tracking_thresh)
				{
					//this->current_mode = robot_mode::detection;
					this->detection_span = 2;
					this->false_tracking_counter = 0;
					ROS_DEBUG("Switch back to detection, %f corners, %f error distance", this->corners.total(), temp_tracking_err);
				}
			}
		}
		this->last_event_time = this->temp_e.ts.toNSec()/1000000000;
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
	double distance = std::sqrt(std::pow(this->random_initial_center.x - (((double)this->sensor_width_)/2.0),2) + std::pow(this->random_initial_center.y - (((double)this->sensor_height_)/2.0),2));
	if (distance >= this->center_offset_threshold)
	{
		this->cmd_vel_twist.linear.x = (this->random_initial_center.x - (((double)this->sensor_width_)/2.0)) * 0.03 / distance;
		this->cmd_vel_twist.linear.y = (this->random_initial_center.y - (((double)this->sensor_height_)/2.0)) * 0.03 / distance;
	}
	else
	{
		this->current_mode=robot_mode::rotate;
		this->cmd_vel_twist.linear.x = 0;
		this->cmd_vel_twist.linear.y = 0;
	}		
	cmd_vel_pub.publish(this->cmd_vel_twist);
	}
	else if (this->current_mode==robot_mode::tracking)
	{
	double distance = std::sqrt(std::pow(this->object_center.x - (((double)this->sensor_width_)/2.0),2) + std::pow(this->object_center.y - (((double)this->sensor_height_)/2.0),2));
	double target_velocity = this->k_p * distance;
	if (target_velocity > this->velocity)
	{
		target_velocity = this->velocity;
	}
	if (distance >= this->center_offset_threshold)
	{
		this->cmd_vel_twist.linear.x = (this->object_center.x - (((double)this->sensor_width_)/2.0)) * target_velocity / distance;
		this->cmd_vel_twist.linear.y = (this->object_center.y - (((double)this->sensor_height_)/2.0)) * target_velocity / distance;
	}
	else
	{
		this->current_mode=robot_mode::rotate;
		this->cmd_vel_twist.linear.x = 0;
		this->cmd_vel_twist.linear.y = 0;
	}		
	cmd_vel_pub.publish(this->cmd_vel_twist);
	}
	else if (this->current_mode==robot_mode::rotate)
	{
		this->ee_orientation();
		cmd_rotate_ee_pub.publish(this->orientation_angle);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		this->current_mode = robot_mode::pickup;
	}	  
	else if (this->current_mode==robot_mode::pickup)
	{
		if (!this->pickup_status)
		{
			ros::service::call("ur_pickup", this->empty_service);
			this->pickup_status = true;
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
		this->corners_var[i] = this->corner_var_constant;
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
		return min_distance_idx;
	
	return -1;
}

void Visual_Servoing::update_corners(dvs_msgs::Event new_event)
{
	int association = this->find_corner_association(new_event);
	if (association!=-1)
	{
		//update other corners with same movement but with higher variance
		for (int i=0 ; i<this->corners.total() ; i++)
		{
			if (i!=association)
			{
			this->corners.at<cv::Point>(i).x  = ( (this->new_event_var + this->corners_var[i]) * this->corners.at<cv::Point>(i).x + this->corners_var[i] * (new_event.x - this->corners.at<cv::Point>(association).x)) / (this->corners_var[i] + this->new_event_var);
			this->corners.at<cv::Point>(i).y  = ( (this->new_event_var + this->corners_var[i]) * this->corners.at<cv::Point>(i).y + this->corners_var[i] * (new_event.y - this->corners.at<cv::Point>(association).y)) / (this->corners_var[i] + this->new_event_var);
			this->corners_var[i] = (this->corners_var[i] * this->new_event_var) / (this->corners_var[association] + this->new_event_var);
			}
		}
		this->corners.at<cv::Point>(association).x  = (this->new_event_var * this->corners.at<cv::Point>(association).x + this->corners_var[association] * new_event.x) / (this->corners_var[association] + this->new_event_var);
		this->corners.at<cv::Point>(association).y  = (this->new_event_var * this->corners.at<cv::Point>(association).y + this->corners_var[association] * new_event.y) / (this->corners_var[association] + this->new_event_var);
		this->corners_var[association] = (this->corners_var[association] * this->new_event_var) / (this->corners_var[association] + this->new_event_var);	  
	}
}

Visual_Servoing::~Visual_Servoing() {
	// TODO Auto-generated destructor stub
}

}
