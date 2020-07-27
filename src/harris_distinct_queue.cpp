#include "harris_distinct_queue.h"


namespace slip_detection_davis
{

HarrisDistinctQueue::HarrisDistinctQueue(int window_size, int queue_size, bool use_polarity) :
		HarrisLocalEventQueues(window_size, queue_size)
{
  // create queues
  const int polarities = use_polarity ? 2 : 1;
  const int num_queues = sensor_width_*sensor_height_ * polarities;

  queues_ = std::vector<HarrisFixedDistinctQueue>
            (num_queues, HarrisFixedDistinctQueue(2*window_size+1, queue_size));
}

HarrisDistinctQueue::~HarrisDistinctQueue()
{
}

bool HarrisDistinctQueue::isFull(int x, int y, bool pol) const
{
  return queues_[getIndex(x, y, pol)].isFull();
}

void HarrisDistinctQueue::newEvent(int x, int y, bool pol)
{
  // update neighboring pixels
  for (int dx=-window_size_; dx<=window_size_; dx++)
  {
    for (int dy=-window_size_; dy<=window_size_; dy++)
    {
      // in limits?
      if (x+dx<0 or x+dx>=sensor_width_ or y+dy<0 or y+dy>=sensor_height_)
      {
        continue;
      }

      // update pixel's queue
      queues_[getIndex(x+dx, y+dy, pol)].addNew(window_size_+dx,
                                                window_size_+dy);
    }
  }
}

Eigen::MatrixXi HarrisDistinctQueue::getPatch(int x, int y, bool pol)
{
  return queues_[getIndex(x, y, pol)].getWindow();
}

int HarrisDistinctQueue::getIndex(int x, int y, bool polarity) const
{
  int polarity_offset = polarity ? sensor_height_*sensor_width_ : 0;
  return y*sensor_width_ + x + polarity_offset;
}

} // namespace
