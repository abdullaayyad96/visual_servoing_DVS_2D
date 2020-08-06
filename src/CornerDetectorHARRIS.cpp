/*
 * CornerDetectorHARRIS.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: raj
 */

#include "CornerDetectorHARRIS.h"

namespace visual_servoing_davis
{

	CornerDetector_HARRIS::CornerDetector_HARRIS( )
	{
		this->update_params(this->queue_size_, this->window_size_, this->harris_threshold_);

		Eigen::VectorXd Dx = Eigen::VectorXd(5);
		Eigen::VectorXd Sx = Eigen::VectorXd(5);
		for (int i=0; i<5; i++)
		{
		Sx[i] = factorial(5 - 1)/
				(factorial(5 - 1 - i) * factorial(i));
		Dx[i] = pasc(i, 5-2) - pasc(i-1, 5-2);
		}
		Sob_G_x = Sx * Dx.transpose();
		
		Sob_G_x = Sob_G_x / Sob_G_x.maxCoeff();

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

	}	



	bool CornerDetector_HARRIS::isCorner (const dvs_msgs::Event &e)
	{
		double score = this->isCorner_Edge_HARRISi(e);

		if (score > this->harris_threshold_)
		{
			return true;
		}
		else
		{
			return false;
		}
		
	}


	double CornerDetector_HARRIS::isCorner_Edge_HARRISi (const dvs_msgs::Event &e)
	{
		// update queues
		queues_->newEvent(e.x, e.y, e.polarity);

		// check if queue is full
		double score = 0;
		if (queues_->isFull(e.x, e.y, e.polarity))
		{
			if (e.x < 4 or e.x > sensor_width_-4 or
					e.y < 4 or e.y > sensor_height_-4)
			{
				return 0; 
			}

			const Eigen::MatrixXi local_patch = queues_->getPatch(e.x, e.y, e.polarity);

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

		 	   score = a*d-b*b - 0.04*(a+d)*(a+d);
		 }

		return score;
	}

	void CornerDetector_HARRIS::update_params(int new_queue_size, int new_windiw_size, double new_harris_threshold)
	{
		this->window_size_ = new_windiw_size;
		this->queue_size_ = new_queue_size;
		this->harris_threshold_ = new_harris_threshold;

		delete(this->queues_);
		this->queues_ = new HarrisDistinctQueue(window_size_, queue_size_, false);
	}



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

	CornerDetector_HARRIS::~CornerDetector_HARRIS() 
	{
		// TODO Auto-generated destructor stub
	}

}
