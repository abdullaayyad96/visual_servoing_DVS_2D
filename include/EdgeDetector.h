/*
 * EdgeDetector.h
 *
 *  Created on: Mar 25, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_EDGEDETECTOR_H_
#define SLIP_DETECTION_DAVIS_SRC_EDGEDETECTOR_H_
#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include "davisdataprocessing.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <Eigen/Dense>

namespace Davis_Data_Processing
{

class EdgeDetector : public davis_data_processing{
public:
	EdgeDetector();
	virtual ~EdgeDetector();

    bool isEdge (const dvs_msgs::Event &e);

private:
    // kernels
      Eigen::MatrixXd Gkernel;

};




}
#endif /* SLIP_DETECTION_DAVIS_SRC_EDGEDETECTOR_H_ */
