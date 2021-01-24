/*
 * CornerDetectorHARRIS.h
 *
 *  Created on: Apr 7, 2019
 *      Author: raj
 */

#ifndef VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORHARRIS_H_
#define VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORHARRIS_H_

#include <deque>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>

#include "harris_local_event_queues.h"
#include "harris_distinct_queue.h"
#include <chrono>

namespace visual_servoing_davis
{

	class CornerDetector_HARRIS{
		public:
			CornerDetector_HARRIS();

			//	CornerDetector_HARRIS( ros::NodeHandle* nodehandle );
			virtual ~CornerDetector_HARRIS();

			double isCorner_Edge_HARRISi (const dvs_msgs::Event &e);
			bool isCorner (const dvs_msgs::Event &e);

			void update_params(int new_queue_size, int new_windiw_size, double new_harris_threshold_);


		private:
			HarrisLocalEventQueues* queues_;
			ros::NodeHandle pnh_;

			Eigen::MatrixXd Gkernel, Sob_G_x, sob_x; // Guassian Kernel, sobel opertor
			
			Eigen::MatrixXd dx, dy; //  // Gradient operated on the patch
			Eigen::MatrixXi window_;
			static const int sensor_width_ = 240;
			static const int sensor_height_ = 180;
			// contains one event
			struct QueueEvent
			{
				int prev, next;
				int x, y;
			};
			std::vector<QueueEvent> queue_;

			// parameters
			int queue_size_ = 25;
			int window_size_ = 4;
			double harris_threshold_ = 4;

			int factorial(int n) const;
			int pasc(int k, int n) const;
	};

}
#endif /* VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORHARRIS_H_ */
