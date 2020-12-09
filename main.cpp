#pragma warning(disable:4996)
//------------------------------------------------------------------------------
// read data from kinect, convert the depth data and color data to cv::mat and save it
// conver the depth data of the specified area to the point cloud  in the camera coordinate
// editor: SUN CHANG JIANG
// date : 2020-12-5
//------------------------------------------------------------------------------
// 

#include "IGrabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL对各种格式的点的支持头文件
#include "imageProcess.h"

int main() {

	IGrabber myGrabber;
	bool isInitSuccess = false;
	//设置是否保存和显示深度和彩色图片
	bool IsSaveImg = true;
	bool IsShowImg = false;
	//pcl viewer初始化及基础参数配置
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(mycloud, fildColor, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"); // 设置点云大小
	imageProcess imageProcessor;
	//传感器初始化
	while (!isInitSuccess) {
		isInitSuccess = myGrabber.SensorInit();
		std::cout << "trying to init" <<std::endl;
	}
	std::cout << "init success" << std::endl;
	Sleep(500);

	//loop

	string basepath = "E:\\Library\\app大赛\\kinect-data\\";
	string begintime = myGrabber.getTime();
	//初始时间
	string savepath = basepath + begintime;
	string command;
	command = "mkdir -p " + savepath;
	system(command.c_str());
	int i = 0;
	while (true) {
		
		clock_t start_time = clock(); //用于计算程序运行时间

		myGrabber.GetImg(IsSaveImg, IsShowImg);
		if (myGrabber.isMappingMatrixempty()) continue;
		if (IsSaveImg) {
			myGrabber.SaveImg(savepath);
		}
		//pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
		imageProcessor.processImageGetGreenRegion(myGrabber.colorImg);
		for (auto& rect : imageProcessor.rectPoint) {
			mycloud = myGrabber.convertDepthToPointXYZ(rect.at(0).x, rect.at(0).y, rect.at(1).x, rect.at(1).y);
		}

		viewer->updatePointCloud<pcl::PointXYZ>(mycloud, fildColor, "cloud");

		viewer->spinOnce(100);
		//
		//文件序列
		string currtime = to_string(i);
		//定义存储路径
		//string savepath = basepath + begintime + "\\";
		pcl::io::savePCDFileASCII(savepath + "\\" + currtime + "test_pcb", *mycloud);

		clock_t end_time = clock();
		cout << " data saved success: " << i << std::endl;
		cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl; //输出程序运行时间
		i++;
	}
	return 0;
}

