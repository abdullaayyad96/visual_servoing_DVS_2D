#pragma once

#include <deque>
#include <Eigen/Dense>

#include "harris_local_event_queues.h"
#include "harris_fixed_distinct_queue.h"

namespace visual_servoing_davis
{

class HarrisDistinctQueue : public HarrisLocalEventQueues
{
public:
	HarrisDistinctQueue(int window_size, int queue_size, bool use_polarity);
  virtual ~HarrisDistinctQueue();

  void newEvent(int x, int y, bool pol=false);
  bool isFull(int x, int y, bool pol=false) const;
  Eigen::MatrixXi getPatch(int x, int y, bool pol=false);

private:
  // data structure
  std::vector<HarrisFixedDistinctQueue> queues_;

  // helper function
  int getIndex(int x, int y, bool polarity) const;

  // constants
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};

} // namespace
