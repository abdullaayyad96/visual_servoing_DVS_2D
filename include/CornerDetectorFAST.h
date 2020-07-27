/*
 * CornerDetectorFAST.h
 *
 *  Created on: Mar 25, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAST_H_
#define SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAST_H_
#pragma once

#include <deque>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include "davisdataprocessing.h"


namespace slip_detection_davis
{
class CornerDetector_Fast : public soft_hand_slip_detection{
public:
	CornerDetector_Fast();
	virtual ~CornerDetector_Fast();
//	 bool isNoise (const dvs_msgs::Event &e);
//	 bool isEdge (const dvs_msgs::Event &e);
    bool isCornerFAST (const dvs_msgs::Event &e);
//    bool isCorner_Edge_HARRISi (const dvs_msgs::Event &e);
//    bool isCornerARC (const dvs_msgs::Event &e);

private:
    // SAE
    Eigen::MatrixXd sae_[2];

    // pixels on circle
    int circle3_[16][2];
    int circle4_[20][2];

    // parameters
    static const int sensor_width_ = 240;
    static const int sensor_height_ = 180;
	  // allocate SAE matrices

};

}
#endif /* SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAST_H_ */
