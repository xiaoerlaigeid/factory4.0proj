#pragma once
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include "stdafx.h"  
#include <fstream>
#include "ImageRenderer.h" 
#include <time.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace cv;

struct vp {
	int x;
	int y;
};

class IGrabber
{
public:
	IGrabber();
	~IGrabber();
	bool SensorInit();
	bool GetImg();
	void SaveImg();
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(const UINT16* depthBuffer, int start_x, int start_y, int end_x, int end_y);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthRegionToPointXYZ(const UINT16* depthBuffer, cv::Mat* colorimg);

	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZGuanFang(UINT16* depthBuffer);
	//将识别区域的深度图像转换为点云
	cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换depth图像到cv::Mat  
	cv::Mat  ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换16bit depth图像
	cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);// 转换color图像到cv::Mat 
	UINT16* GetDepthbuffer();
	bool isMappingMatrixempty();
	UINT16* pBuffer_depth = NULL;					//depth buffer

private:
	int depth_width = 512; //depth图像的大小  
	int depth_height = 424;
	int color_widht = 1920; //color图像的大小 
	int color_height = 1080;
	bool IsSaveImg = true;
	std::vector<ColorSpacePoint> MappingMatrix;
	HRESULT hr;	
	IKinectSensor* m_pKinectSensor = NULL;			// Current Kinect    
	IDepthFrameReader* m_pDepthFrameReader = NULL;	// Depth reader
	IDepthFrameSource* pDepthFrameSource = NULL;	//depth frame source	 
	IColorFrameReader* m_pColorFrameReader = NULL;	// Color reader 
	IColorFrameSource* pColorFrameSource = NULL;	//color frame source
	ICoordinateMapper* m_pMapper = NULL;			// coordinate Mapper	
	cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了  
	cv::Mat depthImg = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image  
	cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image 
	string getTime();
};

