/*
 * CornerDetectorHARRIS.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: raj
 */

#include "CornerDetectorHARRIS.h"

namespace visual_servoing_davis
{

CornerDetector_HARRIS::CornerDetector_HARRIS( ):  Visual_Servoing(){
//		  first_(-1), last_(-1), queue_max_(25){

//			  queues_ = std::vector<CornerDetector_HARRIS>(500,CornerDetector_HARRIS());
//
//
//			  window_ = Eigen::MatrixXi::Constant(9, 9, -1); //window 9 queue 25
//			  queue_.reserve(queue_max_);

	// TODO Auto-generated constructor stub
	 // parameters
	  queue_size_ = 25;
	  window_size_ = 4;
	  kernel_size_ = 5;
	  harris_threshold_ = 8.0;
	  queues_ = new HarrisDistinctQueue(window_size_, queue_size_, true); //4,25

	  Eigen::VectorXd Dx = Eigen::VectorXd(5);
	  Eigen::VectorXd Sx = Eigen::VectorXd(5);
	  for (int i=0; i<5; i++)
	  {
	    Sx[i] = factorial(5 - 1)/
	            (factorial(5 - 1 - i) * factorial(i));
	    Dx[i] = pasc(i, 5-2) - pasc(i-1, 5-2);
//	    std::cout <<"Sx" << std::endl;

//	    std::cout <<Sx[i] << "\t";
//	    std::cout << std::endl;
//	    std::cout <<"Dx" << std::endl;
//
//	    std::cout << Dx[i] << "\t";
//	    std::cout << std::endl;
	  }
	  Sob_G_x = Sx * Dx.transpose();
//		for (int i = 0; i < 5; ++i) {
//		        for (int j = 0; j < 5; ++j)
//		            std::cout << Sob_G_x(i,j) << "\t";
//		        std::cout << std::endl;
//		    }
		Sob_G_x = Sob_G_x / Sob_G_x.maxCoeff();

//		// 5x5 kernel values
//			for (int i = 0; i < 5; ++i) {
//			        for (int j = 0; j < 5; ++j)
//			            std::cout << Sob_G_x(i,j) << "\t";
//			        std::cout << std::endl;
//			    }
//			// 5x5 kernel values
//				for (int i = 0; i < 5; ++i) {
//				        for (int j = 0; j < 5; ++j)
//				            std::cout << Gx_(j,i) << "\t";
//				        std::cout << std::endl;
//				    }

//			  sob_x= Eigen::MatrixXd(5,5);
//			 sob_x<<1,2,3,4,5,
//					 1,2,3,4,5,
//					 14,2,3,4,15,
//					 1,52,3,4,5,
//					 1,2,3,4,44;
//				for (int i = 0; i < 5; ++i) {
//				        for (int j = 0; j < 5; ++j)
//				            std::cout << sob_x(i,j) << "\t";
//				        std::cout << std::endl;
//				    }

	//Mask or Kernel

	// Guassian kernal 5x5
   Gkernel= Eigen::MatrixXd(5,5);

    // Set the papameters
	const double sigma= 1.0; // standard deviation
	int Gwidth=2; int Gheight=2;

	// Generate 5x5 kernel
		for (int i=(-Gheight); i<=Gheight; i++)
		{
			for (int j=(-Gwidth); j<=Gwidth; j++)
			{
				const double s= 2*sigma*sigma;
				const double q= 1/(M_PI*s);
				const double val= q * (exp(-(i*i+j*j)/(s)));
		Gkernel (Gheight+i, Gwidth+j)= val;
		}
			}

	// Normalize the Kernel
	Gkernel /= Gkernel.sum();

	// 5x5 kernel values
//		for (int i = 0; i < 5; ++i) {
//		        for (int j = 0; j < 5; ++j)
//		            std::cout << Gkernel(i,j) << "\t";
//		        std::cout << std::endl;
//		    }
//	 std::cout <<"constucturor harris" << std::endl;


		}



bool CornerDetector_HARRIS::isCorner (const dvs_msgs::Event &e)
{
	return true;}


double CornerDetector_HARRIS::isCorner_Edge_HARRISi (const dvs_msgs::Event &e)
{
	  // update queues
	  queues_->newEvent(e.x, e.y, e.polarity);

	  // check if queue is full
	  double score = 0;//harris_threshold_ - 10.
	  if (queues_->isFull(e.x, e.y, e.polarity))
	  {
		  if (e.x < 4 or e.x > sensor_width_-4 or
		 	      e.y < 4 or e.y > sensor_height_-4)
		 	  {
		 			return 0; //8 - 10.
		 	  }

		 	const Eigen::MatrixXi local_patch = queues_->getPatch(e.x, e.y, e.polarity);

		 	 //const Eigen::MatrixXi local_patch = Eigen::MatrixXi::Constant(9, 9, -1);
		 // Gradient operation on the local patch
		 	 dx = Eigen::MatrixXd::Zero(5, 5);
		 	 dy = Eigen::MatrixXd::Zero(5, 5);
		 	  for (int x=0; x<5; x++)
		 	  {
		 	    for (int y=0; y<5; y++)
		 	    {
		 	      for (int kx=0; kx<5; kx++)
		 	      {
		 	        for (int ky=0; ky<5; ky++)
		 	        {
		 	          dx(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(kx, ky);
		 	          dy(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(ky, kx);
		 	        }
		 	      }
		 	    }
		 	  }

//		 		 std::cout <<" Gradient operation on the local patch" << std::endl;

		 	 // double score = th_C - 10.;

		 // Generating coefficients for Matrix M
			  double a=0., b=0., d=0.; // Matrix M coeffiecients

		 	  for (int x=0; x<5; x++)
		 	  {
		 	    for (int y=0; y<5; y++)
		 	    {
		 	   // Guassian kernel * Gradient x and y
		 	      a += Gkernel(x, y) * dx(x, y) * dx(x, y);
		 	      b += Gkernel(x, y) * dx(x, y) * dy(x, y);
		 	      d += Gkernel(x, y) * dy(x, y) * dy(x, y);
		 	    }
		 	  }
		 //		 std::cout <<" Generating coefficients for Matrix M" << std::endl;

		 //Harris scoring Det(M)-k.Trace(m)^2
		 	   score = a*d-b*b - 0.04*(a+d)*(a+d);
		 	//return std::make_pair(true, true);


		 }

	  return score;
}












//double CornerDetector_HARRIS::isCorner_Edge_HARRISi (const dvs_msgs::Event &e)
//{
//	 std::cout <<"isCorner_Edge_HARRISi" << std::endl;
////	 queues_ = std::vector<CornerDetector_HARRIS> (50);
//		 std::cout <<"isCorner_Edge_HARRISi222" << std::endl;
//
//// Fill the local patch
////	  update neighboring pixels
//	   for (int dx=-4; dx<=4; dx++)
//	   {
//	     for (int dy=-4; dy<=4; dy++)
//	     {
//	       // in limits?
//	       if (e.x+dx<0 or e.x+dx>=sensor_width_ or e.y+dy<0 or e.y+dy>=sensor_height_)
//	       {
//	         continue;
//	       }
//	       queues_[getIndex(e.x+dx, e.y+dy, e.polarity)].addNew(4+dx, 4+dy);
//	     }
//	   }
//		 std::cout <<"Fill the local patch" << std::endl;
//
//
//	   double score = 8 - 10.;
//
//
//if (queues_[getIndex(e.x, e.y, e.polarity)].isFull())
//{
//	 if (e.x < 4 || e.x >= sensor_width_-4 ||
//	      e.y < 4 || e.y >= sensor_height_-4)
//	  {
//			return 8 - 10.;
//	  }
//
//	const Eigen::MatrixXi local_patch = queues_[getIndex(e.x, e.y, e.polarity)].getWindow();
//
//	 //const Eigen::MatrixXi local_patch = Eigen::MatrixXi::Constant(9, 9, -1);
//// Gradient operation on the local patch
//	 dx = Eigen::MatrixXd::Zero(5, 5);
//	 dy = Eigen::MatrixXd::Zero(5, 5);
//	  for (int x=0; x<5; x++)
//	  {
//	    for (int y=0; y<5; y++)
//	    {
//	      for (int kx=0; kx<5; kx++)
//	      {
//	        for (int ky=0; ky<5; ky++)
//	        {
//	          dx(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(kx, ky);
//	          dy(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(ky, kx);
//	        }
//	      }
//	    }
//	  }
//
//		 std::cout <<" Gradient operation on the local patch" << std::endl;
//
//	 // double score = th_C - 10.;
//
//// Generating coefficients for Matrix M
//	  for (int x=0; x<5; x++)
//	  {
//	    for (int y=0; y<5; y++)
//	    {
//	   // Guassian kernel * Gradient x and y
//	      a += Gkernel(x, y) * dx(x, y) * dx(x, y);
//	      b += Gkernel(x, y) * dx(x, y) * dy(x, y);
//	      d += Gkernel(x, y) * dy(x, y) * dy(x, y);
//	    }
//	  }
////		 std::cout <<" Generating coefficients for Matrix M" << std::endl;
//
////Harris scoring Det(M)-k.Trace(m)^2
//	   score = a*d-b*b - 0.04*(a+d)*(a+d);
//	//return std::make_pair(true, true);
//
//
//}
//std::cout <<" score" << std::endl;
//
//return score;
//
//}




//std::pair<bool,bool> CornerDetector_HARRIS::isCorner_Edge_HARRIS (const dvs_msgs::Event &e, double th_E,double th_C)
//{
//
//	 if (e.x < 4 || e.x >= sensor_width_-4 ||
//	      e.y < 4 || e.y >= sensor_height_-4)
//	  {
//			return std::make_pair(false,false);
//	  }
//// Fill the local patch
////	  update neighboring pixels
//	   for (int dx=-4; dx<=4; dx++)
//	   {
//	     for (int dy=-4; dy<=4; dy++)
//	     {
//	       // in limits?
//	       if (e.x+dx<0 or e.x+dx>=sensor_width_ or e.y+dy<0 or e.y+dy>=sensor_height_)
//	       {
//	         continue;
//	       }
//            addNew(4+dx, 4+dy);
//	     }
//	   }
//	//	 std::cout <<"Fill the local patch" << std::endl;
//
//
//	   double score = 8 - 10.;
//
//
//if (isFull())
//{
//
//	const Eigen::MatrixXi local_patch = getWindow();
//
//	 //const Eigen::MatrixXi local_patch = Eigen::MatrixXi::Constant(9, 9, -1);
//// Gradient operation on the local patch
//	 dx = Eigen::MatrixXd::Zero(5, 5);
//	 dy = Eigen::MatrixXd::Zero(5, 5);
//	  for (int x=0; x<5; x++)
//	  {
//	    for (int y=0; y<5; y++)
//	    {
//	      for (int kx=0; kx<5; kx++)
//	      {
//	        for (int ky=0; ky<5; ky++)
//	        {
//	          dx(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(kx, ky);
//	          dy(x, y) += local_patch(x+kx, y+ky)*Sob_G_x(ky, kx);
//	        }
//	      }
//	    }
//	  }
//
////		 std::cout <<" Gradient operation on the local patch" << std::endl;
//
//	 // double score = th_C - 10.;
//
//// Generating coefficients for Matrix M
//	  for (int x=0; x<5; x++)
//	  {
//	    for (int y=0; y<5; y++)
//	    {
//	   // Guassian kernel * Gradient x and y
//	      a += Gkernel(x, y) * dx(x, y) * dx(x, y);
//	      b += Gkernel(x, y) * dx(x, y) * dy(x, y);
//	      d += Gkernel(x, y) * dy(x, y) * dy(x, y);
//	    }
//	  }
////		 std::cout <<" Generating coefficients for Matrix M" << std::endl;
//
////Harris scoring Det(M)-k.Trace(m)^2
//	   score = a*d-b*b - 0.04*(a+d)*(a+d);
//	//return std::make_pair(true, true);
//
//
//}
//return std::make_pair(score>1,score< -0.01);
//
//}

//Eigen::MatrixXi CornerDetector_HARRIS::getWindow() const
//{
//  Eigen::MatrixXi patch = window_;
//  for (int x = 0; x<window_.rows(); x++)
//  {
//    for (int y = 0; y<window_.cols(); y++)
//    {
//      patch(x, y) = (window_(x, y) < 0) ? 0 : 1;
//    }
//  }
//  return patch;
//}
//
//
//int CornerDetector_HARRIS::getIndex(int x, int y, bool polarity) const
//{
//  int polarity_offset = polarity ? sensor_height_*sensor_width_ : 0;
//  return y*sensor_width_ + x + polarity_offset;
//}
//
//
//bool CornerDetector_HARRIS::isFull() const
//{
//  return (queue_.size() >= 25);
//}
//
//
//
//void CornerDetector_HARRIS::addNew(int x, int y)
//{
//  // queue full?
//  if (queue_.size() < queue_max_)
//  {
//    if (window_(x, y) < 0)
//    {
//      // first element?
//      if (queue_.empty())
//      {
//        first_ = 0;
//        last_ = 0;
//
//        QueueEvent qe;
//        qe.prev = -1;
//        qe.next = -1;
//        qe.x = x;
//        qe.y = y;
//        queue_.push_back(qe);
//
//        window_(x, y) = 0;
//      }
//      else
//      {
//        // add new element
//        QueueEvent qe;
//        qe.prev = -1;
//        qe.next = first_;
//        qe.x = x;
//        qe.y = y;
//        queue_.push_back(qe);
//
//        const int place = queue_.size() - 1;
//        queue_[first_].prev = place;
//        first_ = place;
//
//        window_(x, y) = place;
//      }
//    }
//    else
//    {
//      // link neighbors of old event in queue
//      const int place = window_(x, y);
//
//      if (queue_[place].next >= 0 && queue_[place].prev >= 0)
//      {
//        queue_[queue_[place].prev].next = queue_[place].next;
//        queue_[queue_[place].next].prev = queue_[place].prev;
//      }
//
//      // relink first and last
//      if (place == last_)
//      {
//        if (queue_[place].prev >= 0)
//        {
//          last_ = queue_[place].prev;
//          queue_[queue_[place].prev].next = -1;
//        }
//      }
//      queue_[first_].prev = place;
//
//      queue_[place].prev = -1;
//      if (first_ != place)
//      {
//        queue_[place].next = first_;
//      }
//
//      first_ = place;
//    }
//  }
//  else
//  {
//    // is window empty at location
//    if (window_(x, y) < 0)
//    {
//      // update window
//      window_(queue_[last_].x, queue_[last_].y) = -1;
//      window_(x, y) = last_;
//
//      // update queue
//      queue_[queue_[last_].prev].next = -1;
//      queue_[last_].x = x;
//      queue_[last_].y = y;
//      queue_[last_].next = first_;
//      const int second_last = queue_[last_].prev;
//      queue_[last_].prev = -1;
//      queue_[first_].prev = last_;
//      first_ = last_;
//      last_ = second_last;
//    }
//    else
//    {
//      const int place = window_(x, y);
//      if (place != first_)
//      {
//        // update window
//        window_(x, y) = place;
//
//        // update queue
//        if (queue_[place].prev != -1)
//        {
//          queue_[queue_[place].prev].next = queue_[place].next;
//        }
//        if (queue_[place].next != -1)
//        {
//          queue_[queue_[place].next].prev = queue_[place].prev;
//        }
//
//        if (place == last_)
//        {
//          last_ = queue_[last_].prev;
//        }
//
//        queue_[place].prev = -1;
//        queue_[place].next = first_;
//        queue_[first_].prev = place;
//
//        first_ = place;
//      }
//    }
//  }
//}



























int CornerDetector_HARRIS::factorial(int n) const
{
  if (n > 1)
  {
    return n * factorial(n - 1);
  }
  else
  {
    return 1;
  }
}

int CornerDetector_HARRIS::pasc(int k, int n) const
{
  if (k>=0 && k<=n)
  {
    return factorial(n)/(factorial(n-k)*factorial(k));
  }
  else
  {
    return 0;
  }
}

CornerDetector_HARRIS::~CornerDetector_HARRIS() {
	// TODO Auto-generated destructor stub
}

}
