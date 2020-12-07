#ifndef  __IMAGEPROCESS__
#define __IMAGEPROCESS__
#pragma once
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <array>
using namespace cv;
using namespace std;
class imageProcess
{
	private:
		int iLowH = 15;
		int iHighH = 25;
		int iLowS = 43;
		int iHighS = 255;
		int iLowV = 46;
		int iHighV = 255;
		array<int,4> region;
	public:
		imageProcess();
		int processImageGetGreenRegion(Mat& img);

};

#endif // __IMAGEPROCESS__
