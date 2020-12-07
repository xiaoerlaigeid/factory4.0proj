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
#include <pcl/filters/filter.h>
#include <pcl/filters/convolution.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
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
	bool GetImg(bool IsSaveImg, bool IsShowImg);
	void SaveImg(string savepath);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ( int start_x, int start_y, int end_x, int end_y);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthRegionToPointXYZ(const UINT16* depthBuffer, cv::Mat* colorimg);

	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZGuanFang(UINT16* depthBuffer);
	//��ʶ����������ͼ��ת��Ϊ����
	cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// ת��depthͼ��cv::Mat  
	cv::Mat  ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// ת��16bit depthͼ��
	cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);// ת��colorͼ��cv::Mat 
	UINT16* GetDepthbuffer();
	bool isMappingMatrixempty();
	UINT16* pBuffer_depth = NULL;					//depth buffer
	UINT16* depthArray = NULL;
	string getTime();
	cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//ԭʼUINT16 ���ͼ���ʺ�������ʾ��������Ҫ����8λ�ľͿ����ˣ�������ʾ����Ҳ���Ƿǳ��ã��������ԭʼ16λͼ����ɫ���룬�պ��ſ���  
	cv::Mat depthImg = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image  
	cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image 

private:
	int depth_width = 512; //depthͼ��Ĵ�С  
	int depth_height = 424;
	int color_widht = 1920; //colorͼ��Ĵ�С 
	int color_height = 1080;
	std::vector<ColorSpacePoint> MappingMatrix;
	HRESULT hr;	
	IKinectSensor* m_pKinectSensor = NULL;			// Current Kinect    
	IDepthFrameReader* m_pDepthFrameReader = NULL;	// Depth reader
	IDepthFrameSource* pDepthFrameSource = NULL;	//depth frame source	 
	IColorFrameReader* m_pColorFrameReader = NULL;	// Color reader 
	IColorFrameSource* pColorFrameSource = NULL;	//color frame source
	ICoordinateMapper* m_pMapper = NULL;			// coordinate Mapper	

	

};

