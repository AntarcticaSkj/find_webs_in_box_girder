#include "pathPlanning.h"


int main (int argc, char **argv)
{

    ros::init(argc, argv, "sendControl");
 
    PathPlanning PP;

    PP.testCornerNode();
    // ros::Rate loop_rate(PP.rate);
    // while(PP.n_.ok())
    // {
    //     ros::spinOnce();              	 // check for incoming message.
    //     loop_rate.sleep();
    // }
    
    return 0;
}