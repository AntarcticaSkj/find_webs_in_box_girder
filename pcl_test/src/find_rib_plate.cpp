// 用于寻找箱梁点云中的筋板
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>


template<class PointTypePtr>
void findBottomFlange(PointTypePtr cloud)
{
    std::cout << "Read points : " << cloud->points.size() << std::endl;

}

int main (int argc, char **argv)
{

    return 0;
}