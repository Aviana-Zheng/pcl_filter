#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/impl/sampling_surface_normal.hpp>

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/kdtree/io.h>
#include<vector>
#include <pcl/features/normal_3d_omp.h>


using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);			//待滤波点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//滤波后点云

	///读入点云数据
	cout << "->正在读入点云..." << endl;
	pcl::PCDReader reader;
	reader.read("1.pcd", *cloud);
	cout << "\t\t<读入点云信息>\n" << *cloud << endl;
    //clock_t start_ms = clock();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //滤波算法
	pcl::PointCloud<pcl::Normal>				cloud_normals;
	pcl::PointCloud<pcl::PointNormal>		cloud_point_normals;
    pcl::PointCloud<pcl::PointNormal>    Temp_cloud_point_normals;

	//NormalEstimationOMP使用OpenMP标准并行估计每个3D点的局部表面属性，例如表面法线和曲率。
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setRadiusSearch(0.02);
    normalEstimation.setNumberOfThreads(12);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.setInputCloud(cloud);
	normalEstimation.compute(cloud_normals);

    clock_t start_ms = clock();
	cout << "->正在进行SamplingSurfaceNormal..." << endl;
	
	pcl::concatenateFields(*cloud, cloud_normals, cloud_point_normals);//SamplingSurfaceNormal处理的点云
	
	pcl::SamplingSurfaceNormal<pcl::PointNormal> ssn;		//创建滤波器对象
	ssn.setInputCloud(cloud_point_normals.makeShared());
    ssn.setSeed(1);    //Set seed of random function.设置随机函数的种子。
    ssn.setSample(2);  //Set maximum number of samples in each grid 设置每个网格中的最大样本数
    ssn.setRatio(0.1);  //Set ratio of points to be sampled in each grid设置每个网格中要采样的点的比例,比率越大点越多
    ssn.filter(Temp_cloud_point_normals);

	cloud_filtered->resize(Temp_cloud_point_normals.size());
    for (size_t i = 0; i < Temp_cloud_point_normals.size(); i++)//显示关键点把XYZ另存
    {
        cloud_filtered->points[i].x = Temp_cloud_point_normals.points[i].x;
        cloud_filtered->points[i].y = Temp_cloud_point_normals.points[i].y;
        cloud_filtered->points[i].z = Temp_cloud_point_normals.points[i].z;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    clock_t end_ms = clock();
    std::cout << "filter time cost:" << double(end_ms - start_ms) / CLOCKS_PER_SEC << " s" << std::endl;

	///保存下采样点云
	cout << "->正在保存下采样点云..." << endl;
	pcl::PCDWriter writer;
	writer.write("sub.pcd", *cloud_filtered, true);
	cout << "\t\t<保存点云信息>\n" << *cloud_filtered << endl;

	//================================= 滤波前后对比可视化 ================================= ↓

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("befor_filtered and after_filtered"));

	/*-----视口1-----*/
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); //设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
	viewer->setBackgroundColor(0, 0, 0, v1); //设置背景颜色，0-1，默认黑色（0，0，0）
	viewer->addText("befor_filtered", 10, 10, "v1_text", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "befor_filtered_cloud", v1);

	/*-----视口2-----*/
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("after_filtered", 10, 10, "v2_text", v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "after_filtered_cloud", v2);

	/*-----设置相关属性-----*/
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor_filtered_cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "befor_filtered_cloud", v1);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after_filtered_cloud", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "after_filtered_cloud", v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	//================================= 滤波前后对比可视化 ================================= ↑

	return 0;
}
