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

    vector<Point> rectPoint;
    for (int i = 0; i < contours.size(); i++)
    {
        //获得矩形外包围框
        Rect r = boundingRect(Mat(contours[i]));
        //RotatedRect r = minAreaRect(Mat(contours[i]));
        //cout << "contours" << i << "height=" << r.height << "width =" << r.width << "rate =" << ((float)r.width / r.height) << endl;
            cout << "r.x = " << r.x << "  r.y  = " << r.y << "rate =" << ((float)r.width / r.height) << " area = " << ((float)r.width * r.height) << endl;
            Point p1, p2, p3, p4;
            p1.x = r.x;
            p1.y = r.y;
            p2.x = r.x + r.width;
            p2.x = r.y;
            p3.x = r.x + r.width;
            p3.y = r.y + r.height;
            p4.x = r.x;
            p4.y = r.y + r.height;
            rectPoint.push_back(p1);
            rectPoint.push_back(p2);
            rectPoint.push_back(p3);
            rectPoint.push_back(p4);
            //画矩形
            rectangle(img, r, Scalar(0, 0, 255), 2);
    }
    return 1;
}

imageProcess::imageProcess()
{
	array<int, 4> region = { 0 , 0 , 0, 0  };
}
