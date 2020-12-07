#include "imageProcess.h"

int imageProcess::processImageGetGreenRegion(Mat& img)
{
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(img, imgHSV, COLOR_BGR2HSV);
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	Mat imgThresholded;
	cout << iLowH << " " << iHighH << endl;
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	vector<vector<Point> >contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
}

imageProcess::imageProcess()
{
	array<int, 4> region = { 0 , 0 , 0, 0  };
}
