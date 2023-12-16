/**
 * @file pose_estimator.cpp
 * @brief Pose estimator for Jetbot
 *
 * This node is used to estimate the pose of Jetbot.
 * it will fuse the data from IMU and Lidar, and publish the pose to /final_pose topic.
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "boost/thread/mutex.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

// TODO: Add hardware and ICP localizations

ros::Publisher pub_final_pose;
ros::Subscriber sub_lidar;
ros::Subscriber sub_final_vel;
ros::Subscriber sub_imu;

//==============================================================================

float lidar_ranges[360] = {0};
boost::mutex lidar_mutex;

float final_pose[3] = {0, 0, 90}; // x, y, yaw
float odometry_pose[3] = {0, 0, 90};

//==============================================================================

void callback_routine();
void callback_sub_imu(const std_msgs::Float32ConstPtr &msg);
void callback_sub_lidar(const sensor_msgs::LaserScanConstPtr &msg);
void callback_sub_final_vel(const geometry_msgs::TwistConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    ros::Rate loop_rate(50); // 50Hz

    pub_final_pose = n.advertise<geometry_msgs::Pose2D>("/final_pose", 1);
    sub_lidar = n.subscribe("/scan", 1, callback_sub_lidar);
    sub_final_vel = n.subscribe("/final_vel", 1, callback_sub_final_vel);
    sub_imu = n.subscribe("/imu", 1, callback_sub_imu);

    while (ros::ok())
    {
        callback_routine();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void callback_routine()
{
    static uint16_t counter = 0;
    lidar_mutex.lock();
    for (uint16_t i = 0; i < 360; i++)
    {
        if (lidar_ranges[i] > 0)
        {
            counter++;
        }
    }
    lidar_mutex.unlock();
    counter = 0;

    geometry_msgs::Pose2D msg;
    msg.x = final_pose[0];
    msg.y = final_pose[1];
    msg.theta = final_pose[2];
    pub_final_pose.publish(msg);
}

void callback_sub_lidar(const sensor_msgs::LaserScanConstPtr &msg)
{
    lidar_mutex.lock();
    for (uint16_t i = 0; i < 360; i++)
    {
        if (msg->ranges[i] < 0.1)
        {
            lidar_ranges[i] = 0;
            continue;
        }

        if (msg->intensities[i] > 152)
            lidar_ranges[i] = msg->ranges[i];
    }
    lidar_mutex.unlock();
}

void callback_sub_final_vel(const geometry_msgs::TwistConstPtr &msg)
{
    static float prev_vel[3] = {0};

    odometry_pose[0] += (msg->linear.x + prev_vel[0]);
    odometry_pose[1] += (msg->linear.y + prev_vel[1]);

    prev_vel[0] = msg->linear.x;
    prev_vel[1] = msg->linear.y;
}

void callback_sub_imu(const std_msgs::Float32ConstPtr &msg)
{
    static float prev_yaw = 0;

    odometry_pose[2] += (msg->data + prev_yaw);

    prev_yaw = msg->data;
}
