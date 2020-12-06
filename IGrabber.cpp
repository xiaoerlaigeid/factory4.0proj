#include "IGrabber.h"


IGrabber::IGrabber()
{
	
}

IGrabber::~IGrabber()
{
	//SafeRelease(m_pKinectSensor);
	//SafeRelease(m_pDepthFrameReader);
	//SafeRelease(pDepthFrameSource);
	//SafeRelease(m_pColorFrameReader);
	//SafeRelease(pColorFrameSource);
	//SafeRelease(m_pMapper);
	//SafeRelease(pBuffer_depth);
}

bool IGrabber::SensorInit()
{
	int colorWidth = 0;
	int colorHeight = 0;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}
	hr = m_pKinectSensor->Open();
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::Open()");
	}
	// Retrieved Coordinate Mapper
	hr = m_pKinectSensor->get_CoordinateMapper(&m_pMapper);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
	}

	// Retrieved Color Frame Source
	hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
	}
	hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	// Retrieved Depth Frame Source
	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
	}
	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	// Retrieved Infrared Frame Source
	IInfraredFrameSource* infraredSource;
	hr = m_pKinectSensor->get_InfraredFrameSource(&infraredSource);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_InfraredFrameSource()");
	}

	// Retrieved Color Frame Size
	IFrameDescription* colorDescription;
	hr = pColorFrameSource->get_FrameDescription(&colorDescription);
	if (FAILED(hr)) {
		throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
	}

	hr = colorDescription->get_Width(&colorWidth); // 1920
	if (FAILED(hr)) {
		throw std::exception("Exception : IFrameDescription::get_Width()");
	}

	hr = colorDescription->get_Height(&colorHeight); // 1080
	if (FAILED(hr)) {
		throw std::exception("Exception : IFrameDescription::get_Height()");
	}

	//depth reader  
	if (!m_pDepthFrameReader)
	{
		cout << " Can not find the m_pDepthFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}
	//color reader  
	if (!m_pColorFrameReader)
	{
		cout << "Can not find the m_pColorFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}

	SafeRelease(colorDescription);
	SafeRelease(infraredSource);
	return true;
}

bool IGrabber::GetImg()
{
	string basepath = "E:\\Library\\app大赛\\kinect-data\\";

	UINT nBufferSize_depth = 0;
	UINT nBufferSize_coloar = 0;
	RGBQUAD* pBuffer_color = NULL;
	int imageIndex = 0; //图片编号
	IDepthFrame* pDepthFrame = NULL;
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;
		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
		}
		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
			depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			depthImg = ConvertDepthMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
		}
	}
	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16* pBuffer = NULL;
		MappingMatrix.resize(512*424) ;

		hr = pDepthFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}
		if (SUCCEEDED(hr))	{
			hr = pFrameDescription->get_Width(&nWidth);
		}
		if (SUCCEEDED(hr))	{
			hr = pFrameDescription->get_Height(&nHeight);
		}
		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}
		if (SUCCEEDED(hr))	{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}
		if (SUCCEEDED(hr))	{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}
		if (SUCCEEDED(hr))	{
			ICoordinateMapper* pCoordinateMapper;
			m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
			pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, pBuffer, 512 * 424, &MappingMatrix[0]);
			
			//
			//pCoordinateMapper->MapDepthPointToCameraSpace();
		}
		SafeRelease(pFrameDescription);
	}
	SafeRelease(pDepthFrame);
	//for color		
	IColorFrame* pColorFrame = NULL;
	hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	ColorImageFormat imageFormat = ColorImageFormat_None;
	if (SUCCEEDED(hr))
	{
		ColorImageFormat imageFormat = ColorImageFormat_None;
		if (SUCCEEDED(hr))	{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}
		RGBQUAD* m_pColorRGBX = NULL;
		m_pColorRGBX = new RGBQUAD[color_widht * color_height];
		if (SUCCEEDED(hr))	{
			if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
			}
			else if (m_pColorRGBX)
			{
				pBuffer_color = m_pColorRGBX;
				nBufferSize_coloar = color_widht * color_height * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
			}

			else
			{
				hr = E_FAIL;
			}
			colorImg = ConvertMat(pBuffer_color, color_widht, color_height);
		}
		SafeRelease(pColorFrame);
		delete[] m_pColorRGBX;
	}
	namedWindow("depth", 0);
	cv::imshow("depth", depthImg_show);
	namedWindow("color", 0);
	cv::imshow("color", colorImg);
	return true;
}

void IGrabber::SaveImg()
{
	string basepath = "E:\\Library\\app大赛\\kinect-data\\";
	//初始时间
	string begintime = getTime();
	string command;
	command = "mkdir -p " + basepath + begintime;
	system(command.c_str());
	//
	//获取时间戳
	string currtime = getTime();
	//定义存储路径
	string savepath = basepath + begintime + "\\";
	//存储3种图片
	cv::imwrite(savepath + currtime + "_gray.png", depthImg_show);
	cv::imwrite(savepath + currtime +  "_gt.png", depthImg);
	cv::imwrite(savepath + currtime + "_rgb.jpg", colorImg);
	//存储映射关系
	char* g_buffer = new char[512 * 424 * 21];
	//char* g_buffer = (char*)malloc(512 * 424 * 20);
	char* b_temp = g_buffer;
	std::cout << "the matrix size: " << MappingMatrix.size() << std::endl;
	for (size_t i = 0; i < 512 * 424; i++)
	{
		//std::cout<<sizeof(float)<<" "<<sizeof(" ")<< " " <<sizeof("\n") <<std::endl;
		float x = MappingMatrix[i].X;
		float y = MappingMatrix[i].Y;
		//if (x >-100000 && x <100000) {
		if (x > 0 && x < 10000) {
			//std::cout << x << " " << y << std::endl;
		}
		else {
			x = -10000;
			y = -10000;
		}
		sprintf(b_temp, "%09.1lf %09.1lf\n", x, y);
		//std::cout << MappingMatrix[i].X << " " << MappingMatrix[i].Y << std::endl;
		b_temp = b_temp + 20;
	}
	FILE* stream;
	stream = fopen((savepath + currtime + "-align.txt").c_str(), "w");
	fwrite(g_buffer, 20 * 512 * 424, 1, stream);
	fclose(stream);
	Sleep(20);
	delete[] g_buffer;
	//free(g_buffer);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr IGrabber::convertDepthToPointXYZ(const UINT16* depthBuffer,int start_x, int start_y, int end_x, int end_y)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	int cloudWidth = std::abs(start_x - end_x);
	int cloudHeight = std::abs(start_y - end_y);
	cloud->width = static_cast<uint32_t>(cloudWidth);
	cloud->height = static_cast<uint32_t>(cloudHeight);
	cloud->is_dense = false;
	//cloud->points.resize(cloudWidth * cloudHeight);

	std::vector<pcl::PointXYZ*> pclList;
	//pcl::PointXYZ* pt = &cloud->points[0];
	pcl::PointXYZ* pt = NULL;
	for (int i = 0; i < MappingMatrix.size(); i++,pt++) {
		//UINT16 depth = *depthBuffer;
		UINT16 depth = depthImg.data[i];
		ColorSpacePoint sp = MappingMatrix[i];
		//std::cout << "the position of the point "<< i <<" : "<< sp.X <<", "<< sp.Y << std::endl;
		if (sp.X >=start_x && sp.X <= end_x && sp.Y >= start_y && sp.Y <= end_y) {
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(i % cloudWidth), static_cast<float>(i/cloudWidth) };
			//UINT16 depth = depthBuffer[i];

			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			m_pMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

			point.x = cameraSpacePoint.X;
			point.y = cameraSpacePoint.Y;
			point.z = cameraSpacePoint.Z;
			//std::cout << "the position  of the cameraSpacepoint " << i << " : " << point.x << ", " << point.y << ", " <<point.z<<std::endl;
			//*pt = point;
			//pclList.push_back(pt);
			if (abs(point.x + point.y + point.z) < 100000)
			{
				cloud->push_back(point);
			}
			++depthBuffer;
		}
	}
	return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr IGrabber::convertDepthToPointXYZGuanFang( UINT16* depthBuffer)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	cloud->width = static_cast<uint32_t>(depth_width);
	cloud->height = static_cast<uint32_t>(depth_height);
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int y = 0; y < depth_height; y++) {
		for (int x = 0; x < depth_width; x++, pt++) {
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depthBuffer[y * depth_width + x];


			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			m_pMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
			point.x = cameraSpacePoint.X;
			point.y = cameraSpacePoint.Y;
			point.z = cameraSpacePoint.Z;

			*pt = point;
		}
	}

	return cloud;
}
//将深度数据转换为cv::mat
cv::Mat IGrabber::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
		BYTE intensity;
		if (depth <= nMinDepth) {
			intensity = 0;
		}
		else if (depth >= nMaxDepth) {
			intensity = 255;
		}
		else {
			intensity = static_cast<BYTE>(255 * (depth - nMinDepth) / (nMaxDepth - nMinDepth));
		}
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;

		++pBuffer;
	}
	return img;
}

cv::Mat IGrabber::ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;
	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		UINT16 depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
		UINT16 intensity = static_cast<USHORT> (depth);


		*p_mat = intensity;
		p_mat++;
		++pBuffer;
	}
	return img;
}
//将彩色图像转换为cv::Mat
cv::Mat IGrabber::ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;
		++pBuffer;
	}
	return img;

}

UINT16* IGrabber::GetDepthbuffer()
{
	return pBuffer_depth;
}

bool IGrabber::isMappingMatrixempty()
{
	int a = MappingMatrix.size();
	if (a == 0) return true;
	else return false;
}

string IGrabber::getTime()
{
	time_t tt = time(NULL);
	struct tm* stm = localtime(&tt);
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char tmp[32];
	sprintf(tmp, "%04d-%02d-%02d-%02d-%02d-%02d-%03d", 1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday, stm->tm_hour,
		stm->tm_min, stm->tm_sec,sys.wMilliseconds);
	printf(tmp);
	return tmp;
}

