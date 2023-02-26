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
class BoxGirderConstructer
{
public:
    bool displayBoard;
    bool displayLine;
    bool displayPlane;

    BoxGirderConstructer(std::string name, bool _displayBoard=true, bool _displayLine=true, bool _displayPlane=true, float _scale=0.01):
        displayPlane(_displayPlane), displayLine(_displayLine), displayBoard(_displayBoard), scale(_scale)
    {
        v1 = v2 = 0;
        vis.reset(new pcl::visualization::PCLVisualizer(name));
        boardCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (displayBoard)
            generateBoard();
        if (displayLine)
            generateLine();
        if (displayPlane)
            generatePlane();

    }

    void display();
    void saveAsPCD(std::string){}

private:
    float scale; // meter = 0.01, millimeter = 1.0
    static const float boardSpace[];
    static constexpr float L1 = 73840.0;
    static constexpr float L2 = 4870.0;
    static constexpr float H1 = 1800.0;
    static constexpr float H2 = 934.0;
    static constexpr float H3 = 970.0;
    static constexpr float W1 = 890.0;
    static constexpr float W2 = 1214.0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr boardCloud;
    std::vector<std::vector<pcl::PointXYZ>> boardPolygon;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
    int v1, v2;

    void generateLine();
    void generateBoard();
    void generatePlane();
};

const float BoxGirderConstructer::boardSpace[] = {
     3045,  5045,  7045,  9045, 11045, 13045, 15045, 17045, 19045, 21045, 
    23045, 25045, 26965, 27815, 28665, 29515, 31515, 33515, 35515, 37515,
    39515, 41515, 43515, 45515, 47515, 49515, 51515, 53515, 55515, 57515,
    59515, 60410, 61370, 63270, 65170, 67070, 68970, 70553, 72136, 73840
    };


void BoxGirderConstructer::generateBoard()
{
    vis->createViewPort (0.01, 0.0, 1, 1, v2);

    int boardSize = sizeof(boardSpace) / sizeof(float) - 1;
    // std::cout << "boardSize = "  << boardSize << std::endl;

    for (int i = 0; i < boardSize; ++i)
    {
        pcl::PointXYZ tmpPoint;
        std::vector<pcl::PointXYZ> tmpVec(5);

        // lower left
        tmpPoint.x = 0.0;
        tmpPoint.y = boardSpace[i] * scale;
        tmpPoint.z = 0.0;
        boardCloud->points.push_back(tmpPoint);
        tmpVec[0] = tmpPoint;

        // upper left
        if (i < boardSize - 2)
            tmpPoint.z = H1 * scale;
        else // the last two require special treatment
            tmpPoint.z = (H1 - (i + 3 - boardSize) / 3.0 * (H1 - H3)) * scale;
        tmpVec[1] = tmpPoint;
        boardCloud->points.push_back(tmpPoint);

        // upper right
        tmpPoint.x = W1 * scale;
        tmpVec[2] = tmpPoint;
        boardCloud->points.push_back(tmpPoint);

        // middle right
        tmpPoint.z = H2 * scale;
        tmpVec[3] = tmpPoint;
        boardCloud->points.push_back(tmpPoint);

        // lower right
        tmpPoint.x = W2 * scale;
        tmpPoint.z = 0;
        tmpVec[4] = tmpPoint;
        boardCloud->points.push_back(tmpPoint);

        // std::cout << tmpVec[0] << " " << tmpVec[1] << " " << tmpVec[2] << " "  << tmpVec[3] << " " << tmpVec[4] << std::endl;
        boardPolygon.push_back(tmpVec);
    }

    vis->addPointCloud(boardCloud, "board", v2);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"board");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,"board");

    for (int i = 0; i < boardSize; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr boardPolygonP(new pcl::PointCloud<pcl::PointXYZ>); 

        for (int j = 0; j < 5; ++j)
            boardPolygonP->points.push_back(boardPolygon[i][j]);

        vis->addPolygon<pcl::PointXYZ>(boardPolygonP,255,0,0, "board_" + std::to_string(i + 1), v2);
    }
}

void BoxGirderConstructer::generateLine()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr linePolygonP(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointXYZ a, b;
    a.x = W2 / 2 * scale;
    a.y = 0.0;
    a.z = 0.0;
    b.x = W2 / 2 * scale;
    b.y = L1 * scale;
    b.z = 0.0;

    linePolygonP->points.push_back(a);
    linePolygonP->points.push_back(b);
    linePolygonP->points.push_back(a);
    vis->addPolygon<pcl::PointXYZ>(linePolygonP ,0,0,255, "line_1", v2);

    const float line_h[3]  = { 450.0,  900.0, 1350.0};
    const float line_l1[3] = {1000.0, 3045.0, 1000.0};
    const float line_l2[3] = {73840.0, 73840.0, 68970.0};
    for (int i = 0; i < 3; ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr linePolygonP2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ c, d;
        d.x = c.x = 0.0;
        c.y = line_l1[i] * scale;
        d.y = line_l2[i] * scale;
        d.z = c.z = line_h[i] * scale;

        linePolygonP2->points.push_back(c);
        linePolygonP2->points.push_back(d);
        linePolygonP2->points.push_back(c);
        vis->addPolygon<pcl::PointXYZ>(linePolygonP2,0,0,0, "line_" + std::to_string(i + 2), v2);
    }
}

void BoxGirderConstructer::generatePlane()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ a, b, c, d, e, f, g, h, i, j, k, l;
    e.z = e.x = d.x = c.x = a.x = a.y = a.z = 0.0;
    b.x = b.y = 0.0;
    c.z = b.z = H1 * scale;
    c.y = (L1 - L2) * scale;
    e.y = d.y = L1 * scale;
    d.z = H3 * scale;
    planePolygonP1->points.push_back(a);
    planePolygonP1->points.push_back(b);
    planePolygonP1->points.push_back(c);
    planePolygonP1->points.push_back(d);
    planePolygonP1->points.push_back(e);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP1,0,0,255, "plane_1", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP2(new pcl::PointCloud<pcl::PointXYZ>);
    g.x = f.x = W2 * scale;
    f.y = L1 * scale;
    f.z = g.z = g.y = 0.0;
    planePolygonP2->points.push_back(a);
    planePolygonP2->points.push_back(e);
    planePolygonP2->points.push_back(f);
    planePolygonP2->points.push_back(g);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP2,0,0,255, "plane_2", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP3(new pcl::PointCloud<pcl::PointXYZ>);
    i.x = h.x = W1 * scale;
    h.y = 0.0;
    i.y = L1 * scale;
    i.z = h.z = H2 * scale;
    planePolygonP3->points.push_back(f);
    planePolygonP3->points.push_back(g);
    planePolygonP3->points.push_back(h);
    planePolygonP3->points.push_back(i);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP3,0,0,255, "plane_3", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP4(new pcl::PointCloud<pcl::PointXYZ>);
    j.x = l.x = k.x = W1 * scale;
    l.y = 0.0;
    k.y = (L1 - L2) * scale;
    l.z = k.z = H1 * scale;
    j.y = L1 * scale;
    j.z = H3 * scale;
    planePolygonP4->points.push_back(h);
    planePolygonP4->points.push_back(i);
    planePolygonP4->points.push_back(j);
    planePolygonP4->points.push_back(k);
    planePolygonP4->points.push_back(l);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP4,0,0,255, "plane_4", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP5(new pcl::PointCloud<pcl::PointXYZ>);
    planePolygonP5->points.push_back(k);
    planePolygonP5->points.push_back(l);
    planePolygonP5->points.push_back(b);
    planePolygonP5->points.push_back(c);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP5,0,0,255, "plane_5", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr planePolygonP6(new pcl::PointCloud<pcl::PointXYZ>);
    planePolygonP6->points.push_back(j);
    planePolygonP6->points.push_back(k);
    planePolygonP6->points.push_back(c);
    planePolygonP6->points.push_back(d);
    vis->addPolygon<pcl::PointXYZ>(planePolygonP6,0,0,255, "plane_6", v2);
}

void BoxGirderConstructer::display()
{
    vis->setBackgroundColor(255, 255, 255);
    vis->addCoordinateSystem(10, 0.0, 0.0, 0.0);
    while (!vis->wasStopped())
	{
		vis->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

int main()
{
    std::string name = "test";
    BoxGirderConstructer test(name);
    test.display();

    return 0;
}
