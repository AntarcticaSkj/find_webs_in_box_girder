#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <thread>
#include <list>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <pcl_test/PoseAndCorner.h>
#include <nav_msgs/Odometry.h>

// struct Corner
// {
//     float x1;
//     float x2;
//     float y;
// };

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

// struct WebNode
// {
//     float val;
//     bool isCorner;
//     bool isDecteced;
 
//     WebNode(float x = 0.0, bool _isCorner = true, bool _isDetected = false) : 
//         val(x), isCorner(_isCorner), isDecteced(_isDetected) {}
// };



class PathPlanning
{
public:
    // 订阅和发布ROS
    ros::NodeHandle n_;
    double rate;

    PathPlanning():n_("~")
    {
        // ROS 初始化
        rate = 1000; // 10HZ
        pub_twist = n_.advertise<geometry_msgs::Twist>("cmd_vel",100); //向master注册节点信息
        sub_cloudcorner = n_.subscribe("/cloudToWebs/poseAndCorner", 1, &PathPlanning::corner_cbk, this);
        sub_cloudodom = n_.subscribe("/Odometry", 1000, &PathPlanning::odom_cbk, this);

        dummyHead = new WebNode(-1000000.0F);
    }
    void testCornerNode();

private:
    void corner_cbk(const pcl_test::PoseAndCorner& msg);
    void odom_cbk(const nav_msgs::Odometry& msg);
    void pubTwistMsg();

    // 订阅ROS和发布ROS
    ros::Publisher pub_twist;
    ros::Subscriber sub_cloudcorner;
    ros::Subscriber sub_cloudodom;

    // 用于存储的链表
    WebNode* dummyHead;
    WebNode* head;
    WebNode* now;
};


#endif // PATH_PLANNING_H