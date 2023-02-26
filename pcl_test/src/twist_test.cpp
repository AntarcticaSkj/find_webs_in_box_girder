#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
 
int main(int argc,char **argv)
{
	// ros节点初始化      twist_test是节点的名字
	ros::init(argc,argv,"twist_test");
 
	//创建节点句柄
	ros::NodeHandle n;
 
	//创建一个publisher（发布者 名字是turtle_vel_pub） 发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist,队列长度10
	//<>里是消息的数据类型    发一个geometry_msgs::Twist 类型的消息
	// 往/turtle1/cmd_vel话题里发
	// 如果订阅者来不及接收，那发布者就先把消息放在队列里，要是队列满了，就覆盖最老的消息
	ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",10);//向master注册节点信息
 
	//设置循环的频率
	ros::Rate loop_rate(10);
 
	int count = 0;
 	int i = 1;
 
	//封装数据并且发布出去，延时满足频率
	while(ros::ok() ) 
	{
		//初始化geometry_msgs::Twist类型的消息  geometry_msgs::Twist是类，我们创建一个对象叫 vel_msg
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = i * 0.5;
		vel_msg.angular.z = i * 0.2;
		
        i *= -1;
		//发布消息
		// turtle_vel_pub是发布者， publish（）是方法
		turtle_vel_pub.publish(vel_msg);
		// 下面相当于printf
		ROS_INFO("Publsh turtle velocity command[%0.2f m/s,%0.2f rad/s]",
				vel_msg.linear.x,vel_msg.angular.z);
 
		//按照循环频率延时
		loop_rate.sleep();
 
	}
 
	return 0;
}