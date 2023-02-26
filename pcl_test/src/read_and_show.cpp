// load pc ->  do [R T] -> show
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
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
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

void readPCDAndShow(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(path,*cloudin);
    std::cout << "Read points : " << cloudin->points.size() << std::endl;



    // Eigen::Matrix4f offset(Eigen::Matrix4f::Identity()); 
    // offset(1, 3) = -12.;
    // pcl::transformPointCloud (*cloudin, *cloudin, offset);
    // Eigen::Matrix4f offset2(Eigen::Matrix4f::Identity()); 
    // offset2(1, 1) = -1.;
    // offset2(0, 0) = -1.;
    // pcl::transformPointCloud (*cloudin, *cloudin, offset2);
    // pcl::PCDWriter writer;
    // writer.write("/home/sunkejia/3L_xiangliang_re_inversion.pcd", *cloudin);


    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*cloudin, min_p, max_p);
    std::cout << "MIN:\n" << min_p <<  std::endl;
    std::cout << "MAX:\n" << max_p <<  std::endl;
    
    // Eigen::Vector4f centroid;					// 质心
    // pcl::compute3DCentroid(*cloudin, centroid);	// 齐次坐标，（c0,c1,c2,1）
    // std::cout << "centroid:\n" << centroid << std::endl;


    pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudin);
	sor.setLeafSize(0.11f,0.11f,0.11f);
	sor.filter (*cloud_filtered);
    std::cout << "After filter, points : " << cloud_filtered->points.size() << std::endl;
    // cloud_filtered->height = cloud_filtered->points.size();
    // cloud_filtered->width = 1;
    // pcl::PCDWriter writer;
    // writer.write("/home/sunkejia/3L_xiangliang_re.pcd", *cloud_filtered);
    // show
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show"));
    
    viewer->addCoordinateSystem(8, 0.0, 0.0, 0.0);
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud_filtered,255,255,255);
    viewer->addPointCloud(cloud_filtered, cloud_color, "cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main (int argc, char **argv)
{
    readPCDAndShow(argv[1]);

    return 0;
}
