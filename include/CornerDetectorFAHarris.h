/*
 * CornerDetectorFAHarris.h
 *
 *  Created on: Jan 28, 2021
 *      Author: user
 */

/****************************************************************************************
*  FA-Harris corner detection.
*
*  This method will save corner events to `/corner/scene_/fa_harris.txt`
*  with (x, y, t, p). Modify `corner.launch` file to choose scene before `launch`.
*
*  Author: Ruoxiang Li
*  Date: 2019.1.20
**modified by raj

****************************************************************************************/

#ifndef SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAHARRIS_H_
#define SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAHARRIS_H_
#include <Eigen/Dense>
#include <deque>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>


namespace visual_servoing_davis
{
class CornerDetectorFAHarris {
public:
	CornerDetectorFAHarris();
	virtual ~CornerDetectorFAHarris();
    bool isCorner(const dvs_msgs::Event &e);
    bool isCornerWithoutFilter(const dvs_msgs::Event &e);
    bool addNewEventToTimeSurface(const dvs_msgs::Event &e);
    bool isCornerCandidate(const dvs_msgs::Event &e);
    ////
    bool isCornerCandidateRefined(const dvs_msgs::Event &e);
    double getHarrisScore(int img_x, int img_y, bool polarity);
    bool checkPatch(const dvs_msgs::Event &e);
    Eigen::MatrixXi getPatch();

    bool isFiltered(const dvs_msgs::Event &e);

private:
    ///////////////////
    // Filter Parameters
    constexpr static const double filter_threshold_ = 0.05; //50 ms 
    Eigen::MatrixXd sae_latest_[2];

    // Surface of Active Events
    Eigen::MatrixXd sae_[2];

    ///////////////////
    // Circular Breshenham Masks
    const int kSmallCircle_[16][2];
    const int kLargeCircle_[20][2];

    ///////////////////
    Eigen::MatrixXi patch_;

    // pixels on local window 9*9
    int latest_event_local_;

    int window_size_;
    int kernel_size_;
    static const int sensor_width_ = 240;
    static const int sensor_height_ = 180;
    double harris_threshold_;

    // kernels
    Eigen::MatrixXd Gx_, h_;
    int factorial(int n) const;
    int pasc(int k, int n) const;
};

}
#endif /* SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORFAHARRIS_H_ */
