#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

void printDensity(std::vector<int> density, int base)
{
    std::cout << "=== Print Density ===" << std::endl;
    for(auto& i : density)
    {
        std::string s = "";

        for (int j = 1; j < (int)(0.5 + i / base); ++j)
            s += "_";
        
        std::cout << s << std::endl;
    }
    std::cout << "=== Print Done ===" << std::endl;
}


void calDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int count, std::vector<int>& density)
{
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);
    
    float L = max_p.y - min_p.y + 0.001;
    float gap = L / count;

    density.resize(count);

    for (auto& point : cloud->points)
        ++density[(int)((point.y - min_p.y)/ gap)];

}

void mainDensity(int part_num)
{
    int print_avg_star = 40; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/sunkejia/MyData/Point_Cloud/xiangliang_pc/xiangliang.pcd",*cloud);

    std::vector<int> density;

    calDensity(cloud, part_num, density);

    std::cout << "Point Cloud Size = " << cloud->points.size() << std::endl;
    std::cout << "Number of parts = " << part_num << std::endl;
    std::cout << "Base = " << (cloud->points.size() / part_num / print_avg_star) << std::endl;
    printDensity(density, (cloud->points.size() / part_num / print_avg_star));
}

int main(int argc, char **argv)
{
    int part_num = 500;
    if (argc > 1)
        part_num = atoi(argv[1]);

    // R T -> origin cloud 
    // ...
    
    mainDensity(part_num);
}