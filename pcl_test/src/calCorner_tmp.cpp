#include <omp.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_test/CloudPose.h>
#include <geometry_msgs/Twist.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

 
class findWebs
{
public:
    ros::NodeHandle n_;
    double rate;
    pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud;

private:
 
    ros::Publisher pub_;
    ros::Subscriber sub_cloud;
    
    int i;
public:
    findWebs():n_("~")
    {
        originCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        rate = 1; // 1HZ
        i = 1;
        pub_ = n_.advertise<pcl_test::CloudPose>("cloud_pose",10);//向master注册节点信息
        sub_cloud = n_.subscribe("/cloud_registered", 1, &findWebs::cloud_registered_cbk, this);
    }

    void cloud_registered_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::fromROSMsg(*msg, *originCloud);
        std::cout << "point size = " << originCloud->points.size() << std::endl;
        
        /*
        TODO
        */

        pcl_test::CloudPose cp;
        cp.test[0] = 0.1 * i;
        cp.test2[1] = i++;

        pub_.publish(cp);
    }
 
 
 
};//End of class SubAndPub
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "findWebs");
 
    findWebs cC;
 
    ros::Rate loop_rate(cC.rate);
    while(cC.n_.ok())
    {
        ros::spinOnce();              	 // check for incoming message.
        loop_rate.sleep();
    }
 
//    ros::spin ();
 
    return 0;
}