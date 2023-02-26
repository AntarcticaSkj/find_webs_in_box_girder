#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <boost/thread/thread.hpp>

using MY_PCL_TYPE = pcl::PointXYZ;

void visualizeCloudPoints(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud, std::string sName)
{
    // pcl::visualization::CloudViewer viewer(sName);
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped())
    // {}

    // another way 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(sName));

    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<MY_PCL_TYPE> cloud_color(cloud,0,0,255);
    viewer->addPointCloud(cloud,cloud_color,sName);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void visualizeCloudPoints2(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud1, pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud2, std::string sName)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(new pcl::visualization::PCLVisualizer(sName));

    for_visualizer_v->setBackgroundColor(255,255,255);

    int v1(0);
	for_visualizer_v->createViewPort (0.0, 0.0, 0.33, 1, v1);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v1);
	for_visualizer_v->addPointCloud (cloud1,"cloud1",v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud1");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud1");

	int v2(0);
	for_visualizer_v->createViewPort (0.33, 0.0, 0.66, 1, v2);	
	for_visualizer_v->setBackgroundColor (255, 255, 255, v2);
	for_visualizer_v->addPointCloud (cloud2,"cloud2",v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud2");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"cloud2");

	while (!for_visualizer_v->wasStopped())
	{
		for_visualizer_v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void visualizeDetectResult(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud, std::vector<MY_PCL_TYPE>& normal, std::vector<MY_PCL_TYPE>& center, std::vector<std::vector<MY_PCL_TYPE>>& corner)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v(new pcl::visualization::PCLVisualizer("detect result"));

    for_visualizer_v->setBackgroundColor(255,255,255);

    int v1(0);
	for_visualizer_v->createViewPort (0.0, 0.0, 0.5, 1, v1);
	for_visualizer_v->setBackgroundColor (255, 255, 255, v1);
	for_visualizer_v->addPointCloud (cloud,"cloud1",v1);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud1");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud1");
    

    int v2(0);
    // add plane point
    pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud2 (new pcl::PointCloud<MY_PCL_TYPE>);
    for (auto& item : corner)
        for (auto& _point : item)
            cloud2->points.push_back(_point);
    
    for_visualizer_v->createViewPort (0.5, 0.0, 1, 1, v2);	
	for_visualizer_v->setBackgroundColor (255, 255, 255, v2);
	for_visualizer_v->addPointCloud (cloud2,"cloud2",v2);
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud2");
	for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud2");
    
    // link plane point
    for (int index = 0; index < corner.size(); ++index)
    {   
        pcl::PointCloud<MY_PCL_TYPE>::Ptr surface_hull (new pcl::PointCloud<MY_PCL_TYPE>);      
        MY_PCL_TYPE point1, point2, point3, point4;

        surface_hull->points.push_back(corner[index][0]);
        surface_hull->points.push_back(corner[index][1]);
        surface_hull->points.push_back(corner[index][3]);
        surface_hull->points.push_back(corner[index][2]);

        double color = 0.0;
        for_visualizer_v->addPolygon<pcl::PointXYZ>(surface_hull,color,color,color, "plane_" + std::to_string(index),v2);
    }

    int index = 0;
    for (auto& item : center)
        for_visualizer_v->addSphere(item, 0.05, "sphere_" + std::to_string(++index), v2);

    while (!for_visualizer_v->wasStopped())
	{
		for_visualizer_v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void readCloudFromTxt(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud, std::string sFile)
{
    fstream fsread;

    fsread.open(sFile);
    MY_PCL_TYPE pclPnt;
    float dummy;

    while(!fsread.eof())
    {
        fsread>>pclPnt.x>>pclPnt.y>>pclPnt.z>>dummy;
        cloud->points.push_back(pclPnt);
    }
    fsread.close();
}

void readDetectResultFromTxt(std::string sFile, std::vector<MY_PCL_TYPE>& normal, std::vector<MY_PCL_TYPE>& center, std::vector<std::vector<MY_PCL_TYPE>>& corner)
{
    fstream fsread;

    fsread.open(sFile);
    MY_PCL_TYPE pclPnt;
    float dummy;

    while(!fsread.eof())
    {
        MY_PCL_TYPE tmp;
        std::vector<MY_PCL_TYPE> tmp2;
        fsread >> tmp.x >> tmp.y >> tmp.z;
        normal.push_back(tmp);
        fsread >> tmp.x >> tmp.y >> tmp.z;
        center.push_back(tmp);

        fsread >> tmp.x >> tmp.y >> tmp.z;
        tmp2.push_back(tmp);
        fsread >> tmp.x >> tmp.y >> tmp.z;
        tmp2.push_back(tmp);
        fsread >> tmp.x >> tmp.y >> tmp.z;
        tmp2.push_back(tmp);
        fsread >> tmp.x >> tmp.y >> tmp.z;
        tmp2.push_back(tmp);
        corner.push_back(tmp2);
    }
    fsread.close();
}

void saveCloudAsPCD(std::string path, pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud)
{
    pcl::PCDWriter writer;
    writer.write(path, *cloud);
}

void saveCloudAsTxt(std::string path, pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud)
{
    int size = cloud->points.size();
    std::string point_data = "";
    for (int i = 0; i < cloud->points.size(); ++i)
	    point_data = point_data + std::to_string(cloud->points[i].x) + " " + std::to_string(cloud->points[i].y) + " " + std::to_string(cloud->points[i].z)  + "\n";
    
    ofstream OpenF3(path, ios::trunc | ios::out);
    OpenF3 << point_data;
    OpenF3.close(); 
}

void filterCloud(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud, pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud_filtered, float filter_size = 0.1f)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filter_size, filter_size, filter_size);
    sor.filter(*cloud_filtered);
}

void detectGroundOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
    if (cloud->size() > 0)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
  			// seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "Detect done."<<endl;
    }
    else
    {
        cout<<"No data!"<<endl;
    }
}

void detectPlaneRANSACAndShow(pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud)
{
    pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud_p(new pcl::PointCloud<MY_PCL_TYPE>), cloud_f(new pcl::PointCloud<MY_PCL_TYPE>);
    pcl::SACSegmentation<MY_PCL_TYPE> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(1);
    
    int nr_points = (int)cloud->points.size(), det_num = 0;
    pcl::ExtractIndices<MY_PCL_TYPE> extract;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<pcl::PointCloud<MY_PCL_TYPE>> detected_plane;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    while (cloud->points.size() > 0.05 * nr_points)
    {
        std::cout << "Detecting No." << ++det_num << " Plane" << std::endl;

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud.swap(cloud_f);

        int color1 = rand() % 256;
        int color2 = rand() % 256;
        int color3 = rand() % 256;
        std::string name = "plane_" + std::to_string(det_num);
        pcl::visualization::PointCloudColorHandlerCustom<MY_PCL_TYPE> plane_color(cloud_p,color1,color2,color3);
	    viewer->addPointCloud(cloud_p, plane_color, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);

    }
    std::cout << "Detected " << det_num << " Planes" << std::endl;
    
	//保持窗体，知道显示窗体退出
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"find_line_1");

    pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud(new pcl::PointCloud<MY_PCL_TYPE>);
    
    // // // read from txt, transform to pcd and save
    // // readCloudFromTxt(cloud, "/home/sunkejia/Downloads/CaseStudy1_TUB1/CaseStudy1/PointCloud_cs1_dec_x-y-z-localtime6precision.txt");
    // // saveCloudAsPCD("/home/sunkejia/TUB1.pcd", cloud);

    // // read from PCD
    // pcl::io::loadPCDFile("/home/sunkejia/MyData/LIO-SAM/TUB1.pcd",*cloud);
    // // // visual 
    // // visualizeCloudPoints(cloud, "cloud");
    // ROS_INFO("Read done");

    // // filter
    // pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<MY_PCL_TYPE>);
    // filterCloud(cloud, cloud_filtered);
    // // saveCloudAsPCD("/home/sunkejia/TUB1_filtered.pcd", cloud_filtered);
    // // visualizeCloudPoints(cloud_filtered, "cloud_filtered");
    // ROS_INFO("Filter done");

    // // detect groud
    // pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud_filtered_noground(new pcl::PointCloud<MY_PCL_TYPE>);
    // detectGroundOnCloud(cloud_filtered, cloud_filtered_noground);
    // // visualizeCloudPoints(cloud_filtered_noground, "cloud_filtered_noground");
    // saveCloudAsPCD("/home/sunkejia/MyData/LIO-SAM/TUB1_filtered_noground.pcd", cloud_filtered_noground);
    // // saveCloudAsTxt("/home/sunkejia/MyData/LIO-SAM/TUB1_filtered_noground.txt", cloud_filtered_noground);
    // ROS_INFO("Save done");
    pcl::io::loadPCDFile("/home/sunkejia/MyData/Point_Cloud/TUB_pc/TUB1_filtered_noground.pcd",*cloud);

    std::vector<MY_PCL_TYPE> normal, center;
    std::vector<std::vector<MY_PCL_TYPE>> corner;
    readDetectResultFromTxt("/home/sunkejia/MyData/Point_Cloud/TUB_pc/detected_plane.txt", normal, center, corner);

    // detectPlaneRANSACAndShow(cloud);
    visualizeDetectResult(cloud, normal, center, corner);
    return 0;
}
