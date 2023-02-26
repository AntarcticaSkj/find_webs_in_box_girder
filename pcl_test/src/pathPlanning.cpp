#include "pathPlanning.h"

void PRINT_CORNER(WebNode* corner) 
{   
    while(corner->next) 
    {   
        printf("Val = %f, pre_val = %f, next_val = %f\n", 
            corner->val, corner->pre->val, corner->next->val);  
        corner = corner->next;  
    }   
    printf("Val = %f, pre_val = %f\n", corner->val, corner->pre->val); 
}   

// #define PRINT_CORNER(corner) \
// {   \
//     while(corner->next) \
//     {   \
//         printf("Val = %f, pre_val = %f, next_val = %f", 
//             corner->pre->val, corner->val, corner->next->val);  \
//         corner = corner->next;  \
//     }   \
//     printf("Val = %f, pre_val = %f", corner->pre->val, corner->val); \
// }   \

void PathPlanning::testCornerNode()
{
    std::vector<float> in1({10.0, 20.0, 30.0, 40.0, 50.0});
    std::vector<float> in2({10.0, 11.0, 20.0, 30.0, 35.0, 40.0, 50.0});

    std::cout << "flag" << std::endl;
    // 当链表为空时，进行初始化，插入链表
    if (dummyHead->next == nullptr)
    {
        WebNode* in = dummyHead;
        for (auto ff : in1)
        {
            std::cout << "Get in :" << ff << std::endl;
            in->next = new WebNode(ff, in);
            in = in->next;
        }
    }
    std::cout << "flag" << std::endl;
    PRINT_CORNER(dummyHead->next);

    return;
}

/* PoseAndCorner.msg
Header header

float32[9] trans
float32 scale
float32[2] x_short
float32[] corner
*/

void PathPlanning::corner_cbk(const pcl_test::PoseAndCorner& msg)
{

    std::cout << "corner cbk\n";
    printf("Get corner.size(): %lu", msg.corner.size()); // lu = long unsigned int
    printf("Get scale:(%f), x_short:(%f,%f)", msg.scale, msg.x_short[0], msg.x_short[1]);
}

/* From Fast-Lio2
template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    
    ...
}
*/

void PathPlanning::odom_cbk(const nav_msgs::Odometry& msg)
{
    std::cout << "odom cbk\n";
    printf("Get position:(%f, %f, %f)\n", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    printf("Get orientaion:(%f, %f, %f, %f)\n", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);


}

void PathPlanning::pubTwistMsg()
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = 0.2;
    pub_twist.publish(vel_msg);
}