/*
 * CornerDetectorARC.h
 *
 *  Created on: Apr 2, 2019
 *      Author: raj
 */

#ifndef VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORARC_H_
#define VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORARC_H_

#include <deque>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>



namespace visual_servoing_davis
{
class CornerDetector_ARC{
public:
	CornerDetector_ARC();
	virtual ~CornerDetector_ARC();
    bool isCorner (const dvs_msgs::Event &e);
//    bool isCornerFAST (const dvs_msgs::Event &e);
//    bool isCorner_Edge_HARRISi (const dvs_msgs::Event &e);
private:
    // ros::NodeHandle pnh_;

  // Circular Breshenham Masks
  const int kSmallCircle_[16][2];
  const int kLargeCircle_[20][2];

  // Parameters
  constexpr static const double filter_threshold_ = 0.050;
  static const int kSensorWidth_ = 346;
  static const int kSensorHeight_= 260;

  // Surface of Active Events
  Eigen::MatrixXd sae_[2];
  Eigen::MatrixXd sae_latest_[2];
  };
}
#endif /* VISUAL_SERVO_DAVIS_SRC_CORNERDETECTORARC_H_ */
