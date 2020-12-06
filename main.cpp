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

boost::mutex updateModelMutex;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->initCameraParameters();
	return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
}

int main() {
	IGrabber myGrabber;
	bool isInitSuccess = false;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(mycloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小
	while (!isInitSuccess) {
		isInitSuccess = myGrabber.SensorInit();
		std::cout << "trying to init" <<std::endl;
	}
	std::cout << "init success" << std::endl;
	Sleep(5000);
	while (true) {
		cout << "test" << endl;
		//Sleep(5000);
		myGrabber.GetImg();
		if (myGrabber.isMappingMatrixempty()) continue;
		//myGrabber.SaveImg();
		pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
		//mycloud = myGrabber.convertDepthToPointXYZ(myGrabber.GetDepthbuffer(), 200, 200, 700, 1000);
		//Sleep(10);
		mycloud = myGrabber.convertDepthToPointXYZ(myGrabber.pBuffer_depth, 200, 200, 700, 1000);
		//mycloud = myGrabber.convertDepthToPointXYZGuanFang(myGrabber.pBuffer_depth);
		std::cout << "the size of cloud is :" << mycloud->size() << std::endl;
		//if (mycloud->size() < 100) {
		//	std::cout << "the cloud is to small to match" << std::endl;
		//}
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // 按照z字段进行渲染
		viewer->updatePointCloud<pcl::PointXYZ>(mycloud, fildColor, "sample cloud");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小
		viewer->spinOnce(100);
		//while (!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}

			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ>& pcloud1 = *cloud_ptr;
			//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
			//pcloud1.points.push_back(pcl::PointXYZ(10, 10, 80));
			//pcloud1.width = cloud_ptr->size();
			//pcloud1.height = 1;
			//pcloud1.is_dense = true;
			//viewer = simpleVis(cloud_ptr);
			//boost::thread vthread(&viewerRunner, viewer);
			//while (1)//循环抓取深度数据
			//{
			//	pcloud1.clear();
			//	for (int _row = 0; _row < disp.rows; _row++)
			//	{
			//		for (int _col = 0; _col < disp.cols; _col++)
			//		{
			//			float x, y, z;
			//			pcl::PointXYZ ptemp(x, y, z);
			//			pcloud1.points.push_back(ptemp);
			//		}
			//	}
			//	pcloud1.width = cloud_ptr->size();
			//	pcloud1.height = 1;
			//	pcloud1.is_dense = true;
			//	boost::mutex::scoped_lock updateLock(updateModelMutex);
			//	viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
			//	updateLock.unlock();
			//	boost::this_thread::sleep(boost::posix_time::microseconds(100));
			//}
			//vthread.joint();
	}
	return 0;
}