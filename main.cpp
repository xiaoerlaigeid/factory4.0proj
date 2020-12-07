#pragma warning(disable:4996)
//------------------------------------------------------------------------------
// read data from kinect, conver the depth data and color data to cv::mat and save it
// conver the depth data of the specified area to the point cloud  in the camera coordinate
// editor: SUN CHANG JIANG
// date : 2020-12-5
//------------------------------------------------------------------------------
// 

#include "IGrabber.h"
#include <pcl/visualization/pcl_visualizer.h>

//boost::mutex updateModelMutex;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//	// --------------------------------------------
//	// -----Open 3D viewer and add point cloud-----
//	// --------------------------------------------
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	viewer->initCameraParameters();
//	return (viewer);
//}
//
//void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
//{
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100));
//	}
//}

int main() {
	IGrabber myGrabber;
	bool isInitSuccess = false;
	bool isSaveImg = false;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // ����z�ֶν�����Ⱦ
	viewer->addPointCloud<pcl::PointXYZ>(mycloud, fildColor, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud"); // ���õ��ƴ�С

	//��������ʼ��
	while (!isInitSuccess) {
		isInitSuccess = myGrabber.SensorInit();
		std::cout << "trying to init" <<std::endl;
	}
	std::cout << "init success" << std::endl;
	Sleep(500);

	//loop
	while (true) {
		myGrabber.GetImg();
		if (myGrabber.isMappingMatrixempty()) continue;
		if (isSaveImg) {
			myGrabber.SaveImg();
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
		mycloud = myGrabber.convertDepthToPointXYZ(myGrabber.pBuffer_depth, 200, 200, 700, 1000);
		std::cout << "the size of cloud is :" << mycloud->size() << std::endl;
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // ����z�ֶν�����Ⱦ
		viewer->updatePointCloud<pcl::PointXYZ>(mycloud, fildColor, "sample cloud");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // ���õ��ƴ�С
		viewer->spinOnce(100);
	}
	return 0;
}