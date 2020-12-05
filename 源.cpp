#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
	int iLowH = 15;
	int iHighH = 25;
	int iLowS = 43;
	int iHighS = 255;
	int iLowV = 46;
	int iHighV = 255;

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control
	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);



	Mat imgOriginal = imread("1.bmp");
	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
	//Convert the captured frame from BGR to HSV
	//因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
	while(1){
		split(imgHSV, hsvSplit);
		equalizeHist(hsvSplit[2], hsvSplit[2]);
		merge(hsvSplit, imgHSV);
		Mat imgThresholded;
		cout << iLowH << " " << iHighH << endl;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//开操作 (去除一些噪点)
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

		char key = (char)waitKey(300);
		if (key == 27)
			break;
	}

	return 0;
}