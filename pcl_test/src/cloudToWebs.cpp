#include "findWebs.h"

int main(int argc, char **argv)
{
    // =====   
    ros::init(argc, argv, "cloudToWebs");
    ros::NodeHandle n_;
    float intervalLength, octreeSize, centroidNearRetainThre, peak_len,
        avg_len, peak_judge_thre, YShapePositive, YShapeNegative, densityMatchX_thre;
    int octreeRemoveSize, octreeSavePointSize, RunMainPointSizeThre;
    bool ifFilterShape, ifCorrectMapCloudPitch, ifUseXMaxMin;
    n_.param<float>("cloudToWebs/intervalLength",intervalLength, 0.01);
    n_.param<int>("cloudToWebs/octreeRemoveSize",octreeRemoveSize, 3);
    n_.param<int>("cloudToWebs/octreeSavePointSize",octreeSavePointSize, 100);
    n_.param<float>("cloudToWebs/octreeSize",octreeSize, 0.05);

    n_.param<float>("cloudToWebs/centroidNearRetainThre",centroidNearRetainThre,0.95);
    n_.param<int>("cloudToWebs/RunMainPointSizeThre",RunMainPointSizeThre, 4321);
    n_.param<float>("cloudToWebs/peak_len",peak_len, 1.0);
    n_.param<float>("cloudToWebs/avg_len", avg_len, 1.5);
    n_.param<float>("cloudToWebs/peak_judge_thre", peak_judge_thre, 2.5);

    n_.param<bool>("cloudToWebs/ifFilterShape",ifFilterShape,false);
    n_.param<bool>("cloudToWebs/ifCorrectMapCloudPitch",ifCorrectMapCloudPitch,false);
    n_.param<bool>("cloudToWebs/ifUseXMaxMin",ifUseXMaxMin,false);
    n_.param<float>("cloudToWebs/YShapePositive",YShapePositive,20.0);
    n_.param<float>("cloudToWebs/YShapeNegative",YShapeNegative,0.0);
    n_.param<float>("cloudToWebs/densityMatchX_thre",densityMatchX_thre,3.0);
    std::cout << "RunMainPointSizeThre: " << RunMainPointSizeThre <<  std::endl;
    // ===== 

    FindWebs cC;
    cC.initParam(intervalLength, octreeSize, centroidNearRetainThre, peak_len,
        avg_len, peak_judge_thre, YShapePositive, YShapeNegative, densityMatchX_thre,
        octreeRemoveSize, octreeSavePointSize, RunMainPointSizeThre, ifFilterShape,
        ifCorrectMapCloudPitch, ifUseXMaxMin);
 

    // cC.startfindBottomFlange();
    // std::cout << "Find Rib Plate done!" << std::endl;
    cC.startRunMain();

    ros::Rate loop_rate(cC.rate);
    while(cC.n_.ok())
    {
        ros::spinOnce();              	 // check for incoming message.
        loop_rate.sleep();
    }
 

    return 0;
}