#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    ros::Rate loop_rate(50); // 50Hz

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}