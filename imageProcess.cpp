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
    //������ (ȥ��һЩ���)
    Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    Mat element1 = getStructuringElement(MORPH_RECT, Size(21, 21));
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element1);

	vector<vector<Point> >contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    rectPoint.clear();
    for (int i = 0; i < contours.size(); i++)
    {
        //��þ������Χ��
        Rect r = boundingRect(Mat(contours[i]));
        //RotatedRect r = minAreaRect(Mat(contours[i]));
        //cout << "contours" << i << "height=" << r.height << "width =" << r.width << "rate =" << ((float)r.width / r.height) << endl;
        //cout << "r.x = " << r.x << "  r.y  = " << r.y << "rate =" << ((float)r.width / r.height) << " area = " << ((float)r.width * r.height) << endl;
        Point upLeft, downRight;
        upLeft.x = r.x;
        upLeft.y = r.y;
        downRight.x = r.x + r.width;
        downRight.y = r.y + r.height;

        vector<Point> vecPoint;
        vecPoint.push_back(upLeft);
        vecPoint.push_back(downRight);
        rectPoint.push_back(vecPoint);
        //������
        //rectangle(img, r, Scalar(0, 0, 255), 2);
    }
    return 1;
}

imageProcess::imageProcess()
{
	array<int, 4> region = { 0 , 0 , 0, 0  };
}
