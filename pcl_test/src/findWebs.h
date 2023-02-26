#ifndef CAL_CORNER_H
#define CAL_CORNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <thread>

#include <math.h>
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_test/CloudPose.h>
#include <pcl_test/PoseAndCorner.h>
// #include <geometry_msgs/Twist.h>  // NEED?

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>
using MyPoint = pcl::PointXYZ;
using MyPointCloudPtr = pcl::PointCloud<MyPoint>::Ptr;


struct PCAResult
{
    Eigen::Vector4f centroid;
    Eigen::Matrix3f eigenvector;
    Eigen::Vector3f eigenvalue;
};

struct WebNode
{
    float val;
    WebNode *pre;
    WebNode *next;
    bool isCorner;
    bool isDecteced;
    
    WebNode(float x = 0.0, WebNode *_pre = nullptr, WebNode *_next = nullptr, bool _isCorner = true, bool _isDetected = false) : 
        val(x), pre(_pre), next(_next), isCorner(_isCorner), isDecteced(_isDetected) {}
};

class FindWebs
{
public:
    // for test
    float sz = -1.0;
    bool show = false;


    // 阈值 ======================
    int RunMainPointSizeThre; // 运行runMain程序所需的最小点云数量要求

    // 分类体素滤波用
    int octreeRemoveSize; // 八叉树去噪时的阈值，若某个点云的邻居数量小于这个值，将会被认作噪声。
    int octreeSavePointSize; // 八叉树体素滤波时每个格子保留的点云数量
    float octreeSize; // 八叉树滤波时的尺寸大小

    // 密度-峰值法用
    float intervalLength; // 计算密度时的区间长度
    float peak_len; 
    int peak_interval_len_min; // 峰所占区间的最小数量 TODE
    int peak_interval_len_max; // 峰所占区间的最大数量 TODE
    float peak_judge_thre;
    float avg_len;
    float densityMatchX_thre;

    // 直通滤波用
    bool ifCentroidNearRetainShape; // 是否按照质心点附近比例进行shape
    float centroidNearRetainThre; //
    bool ifFilterShape; // 是否按照形状进行filter
    float YShapePositive; // 长轴长度
    float YShapeNegative; // 长轴长度
    // 阈值 ======================
    bool ifCorrectMapCloudPitch;
    bool ifUseXMaxMin;
    std::mutex mtx;



    // 订阅和发布ROS
    ros::NodeHandle n_;
    double rate;
    ros::Publisher pub_marker_webs;
    ros::Publisher pub_map;
    ros::Publisher pub_marker_webslines;
    ros::Subscriber sub_cloud;
    void cloud_registered_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
	    mtx.lock();
        pcl::fromROSMsg(*msg, *perOriginCloud);
        *originCloud += *perOriginCloud;
        mtx.unlock();
    }
    void outputResultFile();
    void setSubAndPub()
    {
        sub_cloud = n_.subscribe("/cloud_registered", 10000, &FindWebs::cloud_registered_cbk, this);
        pub_marker_webs = n_.advertise<visualization_msgs::Marker>("visualization_marker_webs", 10000);//initialize marker publisher
        pub_marker_webslines = n_.advertise<visualization_msgs::Marker>("visualization_marker_lines", 10000);//initialize marker publisher
        pub_map = n_.advertise<sensor_msgs::PointCloud2>("final_map", 10000);
    }
    FindWebs():n_("FindWebs")
    {
        // ROS 初始化
    
    
        perOriginCloud.reset(new pcl::PointCloud<MyPoint>());
        rate = 1000; // 10HZ
        result_path = std::string(ROOT_DIR) + "output/" + std::to_string(clock()) + "_result.txt";
        setSubAndPub();
        // 阈值初始化(需要输入) xiangliang_sim
        // intervalLength = 0.01; // 计算密度时的区间长度，单位(m)
        // octreeRemoveSize = 3;
        // octreeSavePointSize = 100;
        // octreeSize = 0.05;

        // centroidNearRetainThre = 0.95;
        // RunMainPointSizeThre = 20000;
        // peak_len = 1.0; // 1m之内只能有一个peak
        // avg_len = 3; // 求1m区间内的密度平均值
        // peak_judge_thre = 5; // 密度 > thre * 密度平均值的区间认为是peak所在区间

        // ifFilterShape = false; // 是否按照形状进行filter
        // YShapePositive = 20.0; // 长轴长度
        // YShapeNegative = 0.0; // 长轴长度
        // densityMatchX_thre = 3;

        // // 阈值初始化(需要输入) xiangliang_real
        // intervalLength = 0.01; // 计算密度时的区间长度，单位(m)
        // octreeRemoveSize = 3;
        // octreeSavePointSize = 100;
        // octreeSize = 0.05;

        // centroidNearRetainThre = 0.95;
        // RunMainPointSizeThre = 300000;
        // peak_len = 1.0; // 1m之内只能有一个peak
        // avg_len = 1.5; // 求1m区间内的密度平均值
        // peak_judge_thre = 2.5; // 密度 > thre * 密度平均值的区间认为是peak所在区间

        // ifFilterShape = true; // 是否按照形状进行filter
        // YShapePositive = 20.0; // 长轴长度
        // YShapeNegative = 0.0; // 长轴长度

        // densityMatchX_thre = 1.5

        mapCloud_x_min = 0.0;
        mapCloud_x_max = 0.0;
        mapCloud_z_min = -1.14;
        mapCloud_z_max = 0.0;


  

        // 参数初始化
        ones_y << 0, 1, 0;
        T_target_origin = Eigen::Matrix4f::Identity();
        scale_originToTarget = 1.0;  // DE
        peak_interval_len_min = peak_len / intervalLength - 4;
        peak_interval_len_max = peak_interval_len_min + 4;

        originCloud.reset(new pcl::PointCloud<MyPoint>());
        // filteredCloud.reset(new pcl::PointCloud<MyPoint>());
        correctedCloud.reset(new pcl::PointCloud<MyPoint>());
        downsamplingCloud.reset(new pcl::PointCloud<MyPoint>());
        removeBadPointCloud.reset(new pcl::PointCloud<MyPoint>());
        mapCloudDense.reset(new pcl::PointCloud<MyPoint>());
        mapCloud.reset(new pcl::PointCloud<MyPoint>());

        // 报错终止初始化
        NO_ERROR = true;

        // 后处理初始化
        webs_detect_times = {0,0,0,0,0,0,0,0,0,0};
        all_webs.resize(10);

    }
    void startRunMain() { 
        std::cout << "Start run Main" << std::endl;
        thread_main = std::thread(&FindWebs::runMain, this);}// 启动多线程
    void runMain();
    void initParam(float intervalLength_, float octreeSize_, float centroidNearRetainThre_, float peak_len_,
        float avg_len_, float peak_judge_thre_, float YShapePositive_, float YShapeNegative_, float densityMatchX_thre_, 
        int octreeRemoveSize_, int octreeSavePointSize_, int RunMainPointSizeThre_, bool ifFilterShape_,
        bool ifCorrectMapCloudPitch_, bool ifUseXMaxMin_)
    {
        intervalLength = intervalLength_;
        octreeSize = octreeSize_;
        centroidNearRetainThre = centroidNearRetainThre_;
        peak_len = peak_len_;
        avg_len = avg_len_;
        peak_judge_thre = peak_judge_thre_;
        YShapePositive = YShapePositive_;
        YShapeNegative = YShapeNegative_;
        densityMatchX_thre = densityMatchX_thre_;
        octreeRemoveSize = octreeRemoveSize_;
        octreeSavePointSize = octreeSavePointSize_;
        RunMainPointSizeThre = RunMainPointSizeThre_;
        ifFilterShape = ifFilterShape_;
        ifCorrectMapCloudPitch = ifCorrectMapCloudPitch_;
        ifUseXMaxMin = ifUseXMaxMin_;
        std::cout << "RunMainPointSizeThre: " << RunMainPointSizeThre <<  std::endl;
    }
    // TO REVISE
    void getProcessResult(Eigen::Matrix4f& _T_target_origin, float& _scale, 
        std::vector<float>& corner, int _x_l, int _x_r)
    {
        _T_target_origin = T_target_origin;
        _scale = scale_originToTarget;
        corner = peak_y_map;
        _x_l = mapCloud_x_min;
        _x_r = mapCloud_x_max;
    }
    void printCornerInOriginCloud();


    void saveCloudAsPCD(MyPointCloudPtr cloud, int time, int index)
    {
        cloud->height = cloud->points.size();
        cloud->width = 1;
        std::string path = std::string(ROOT_DIR) + "output/" + std::to_string(time) + "_" + std::to_string(index) + ".pcd";
        pcl::PCDWriter writer;
        writer.write(path, *cloud);
    }

    void startfindBottomFlange()
    {
        std::string path = "/home/sunkejia/MyData/Point_Cloud/xiangliang_pc/xiangliang_corrected.pcd";
        MyPointCloudPtr cloudin(new pcl::PointCloud<MyPoint>);
        pcl::io::loadPCDFile(path,*cloudin);

        findBottomFlange(cloudin);
    }



 
private:
    // void removeGroud();
    void runPostProcess();
    void octreeRemovePoints(MyPointCloudPtr, MyPointCloudPtr );
    void filterCloud(MyPointCloudPtr, MyPointCloudPtr&);
    void getDensityY(MyPointCloudPtr, std::vector<int>&, MyPoint, MyPoint);
    void getDensityX(MyPointCloudPtr, std::vector<int>&, MyPoint, MyPoint);
    void getDensityZ(MyPointCloudPtr, std::vector<int>&, MyPoint, MyPoint);
    void showCloud2(MyPointCloudPtr , MyPointCloudPtr);
    void filterAndCorrect(MyPointCloudPtr, MyPointCloudPtr&);
    void correctPC(MyPointCloudPtr, MyPointCloudPtr&, Eigen::Matrix4f&, bool);
    PCAResult calPCA(MyPointCloudPtr);
    void calRotation(Eigen::Vector3f, Eigen::Vector3f, double&, Eigen::Vector3f&);
    Eigen::Matrix4f RodriguesMatrixTranslation(Eigen::Vector3f, double);
    void calPCAResultCoordinateForDisplay(const PCAResult&, MyPointCloudPtr);
    void densityMatchY(MyPointCloudPtr, std::vector<float>&);
    void densityMatchX(MyPointCloudPtr, std::vector<float>&);
    void densityMatchZ(MyPointCloudPtr, std::vector<float>&);
    void downsampling(MyPointCloudPtr, float, MyPointCloudPtr&);
    // void peakToCorner(std::vector<int>&);
    void correctMapCloudDense();
    void correctMapCloudToY();
    bool checkTransConvergent(Eigen::Matrix4f, Eigen::Matrix4f);
    void findBottomFlange(MyPointCloudPtr);
    void pubMap();
    void pubVisualMarker();
    void mergeWebs();
    void correctMapCloudPitch();

    // MyPointCloudPtr targetCloud;
    MyPointCloudPtr perOriginCloud;
    MyPointCloudPtr originCloud;
    MyPointCloudPtr downsamplingCloud;
    MyPointCloudPtr removeBadPointCloud;
    MyPointCloudPtr filteredCloud;
    MyPointCloudPtr correctedCloud;
    MyPointCloudPtr mapCloudDense;
    MyPointCloudPtr mapCloud;

    Eigen::Vector3f ones_y; // 矫正后的长轴方向，默认为[0;1;0]，已写死。


    // 报错终止(test)
    bool NO_ERROR;

    // runMain线程
    std::thread thread_main;

    // 用于记录上一次的计算数据，节省时间
    float mapCloud_x_min, mapCloud_x_max; // mapCloud点云x坐标的极大值、极小值
    float mapCloud_z_min, mapCloud_z_max; // mapCloud点云z坐标的极大值、极小值
    float mapCloud_y_min, mapCloud_y_max; // mapCloud点云z坐标的极大值、极小值
    // Eigen::Vector4f lastCentroid;

    int runTimes = 0;
    bool isFirstPCAcorrect = true;
    bool isPCAIntialized = false;  // 是否初始化完成

    // 最终的计算结果
    Eigen::Matrix4f T_target_origin; // 原始点云->模板的旋转矩阵
    // Eigen::Matrix4f T_target_origin_before;
    float scale_originToTarget; // 原始点云 -> 模板的缩放系数
    std::vector<float> peak_y_map; // Map点云中隔板的y坐标
    std::vector<float> peak_x_map; // Map点云中壁面的x坐标
    std::vector<float> peak_z_map; // Map点云中壁面的z坐标

    // 用于存储的链表
    WebNode* dummyHead; // 头
    WebNode* now;

    // 用于后处理
    std::vector<std::vector<float>> all_webs;
    std::vector<int> webs_detect_times;
    std::vector<float> webs_avg;
    std::vector<float> webs_std;

    std::string result_path;

    float h_imu = -0.75;
};


# endif // CAL_CORNER_H
