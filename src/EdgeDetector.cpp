/*
 * EdgeDetector.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: raj
 */

#include "EdgeDetector.h"
namespace Davis_Data_Processing
{
EdgeDetector::EdgeDetector() {
	// TODO Auto-generated constructor stub


//Mask or Kernel

// Guassian kernal 5x5
	// Set the papameters
	double sigma= 1.0; // standard deviation
	int Gwidth=2;
	int Gheight=2;
	Gkernel(Gheight,Gwidth);
	// Generate 5x5 kernel
	for (int i=-Gheight; i<=Gheight; i++)
	{
		for (int j=-Gwidth; j<=Gwidth; j++)
		{
	double s= 2*sigma*sigma;
	double q= 1/(M_PI*s);
	Gkernel [Gheight+i][Gwidth+j]= q * (exp(-(i*i+j*j)/(s)));
	}
		}
	// Normalize the Kernel
	Gkernel /= Gkernel.sum();

	for (int i = 0; i < 5; ++i) {
	        for (int j = 0; j < 5; ++j)
	            std::cout << Gkernel[i][j] << "\t";
	        std::cout << std::endl;
	    }
	}



//	Apply a guassian blur









}

EdgeDetector::~EdgeDetector() {
	// TODO Auto-generated destructor stub
}

}
