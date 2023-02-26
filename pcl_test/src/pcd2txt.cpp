// load .pcd -> ouput .txt
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

using MY_PCL_TYPE = pcl::PointXYZ;

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


int main(int argc,char** argv)
{
    pcl::PointCloud<MY_PCL_TYPE>::Ptr cloud(new pcl::PointCloud<MY_PCL_TYPE>);
    
    pcl::io::loadPCDFile(argv[1],*cloud);

    saveCloudAsTxt(argv[2], cloud);
    return 0;
}
