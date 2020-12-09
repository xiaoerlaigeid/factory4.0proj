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
		int iLowH = 38;
		int iHighH = 60;
		int iLowS = 67;
		int iHighS = 149;
		int iLowV = 113;
		int iHighV = 245;

	public:
		imageProcess();
		int processImageGetGreenRegion(Mat& img);
		vector<vector<Point>> rectPoint;
};

#endif // __IMAGEPROCESS__
