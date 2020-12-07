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
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�

int main() {

	IGrabber myGrabber;
	bool isInitSuccess = false;
	//�����Ƿ񱣴����ʾ��ȺͲ�ɫͼƬ
	bool IsSaveImg = true;
	bool IsShowImg = false;
	//pcl viewer��ʼ����������������
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(mycloud, "z"); // ����z�ֶν�����Ⱦ
	viewer->addPointCloud<pcl::PointXYZ>(mycloud, fildColor, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"); // ���õ��ƴ�С

	//��������ʼ��
	while (!isInitSuccess) {
		isInitSuccess = myGrabber.SensorInit();
		std::cout << "trying to init" <<std::endl;
	}
	std::cout << "init success" << std::endl;
	Sleep(500);

	//loop

	string basepath = "E:\\Library\\app����\\kinect-data\\";
	string begintime = myGrabber.getTime();
	//��ʼʱ��
	string savepath = basepath + begintime;
	string command;
	command = "mkdir -p " + savepath;
	system(command.c_str());
	int i = 0;
	while (true) {
		
		clock_t start_time = clock(); //���ڼ����������ʱ��

		myGrabber.GetImg(IsSaveImg, IsShowImg);
		if (myGrabber.isMappingMatrixempty()) continue;
		if (IsSaveImg) {
			myGrabber.SaveImg(savepath);
		}
		//pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());

		mycloud = myGrabber.convertDepthToPointXYZ(200, 200, 700, 700);


		viewer->updatePointCloud<pcl::PointXYZ>(mycloud, fildColor, "cloud");

		viewer->spinOnce(100);
		//
		//�ļ�����
		string currtime = to_string(i);
		//����洢·��
		//string savepath = basepath + begintime + "\\";
		pcl::io::savePCDFileASCII(savepath + "\\" + currtime + "test_pcb", *mycloud);

		clock_t end_time = clock();
		cout << " data saved success: " << i << std::endl;
		cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl; //�����������ʱ��
		i++;
	}
	return 0;
}

