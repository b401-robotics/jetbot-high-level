/**
 * @file motor.cpp
 * @brief Motor control for Jetbot
 *
 * This node is used to control the motor of Jetbot.
 * basically it will subscribe to /cmd_vel topic, using inverse kinematics and publish the motor speed to /jetbot_motors/cmd_raw topic.
 */

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"

#define DISTANCE_WHEEL_TO_WHEEL 6.31f // cm

//==============================================================================

ros::Publisher pub_motor_speed;
ros::Publisher pub_feedback_motor;
ros::Subscriber sub_cmd_vel;

//==============================================================================

int16_t motor_speed[2] = {0, 0};
float cmd_vel[3] = {0, 0, 0}; // x,y, yaw
uint8_t is_keyboard_interrupt = 0;
double last_time_keyboard_interrupt = 0;

int8_t reverse_speed = 0;

//==============================================================================

void callback_motor_routine();
void callback_sub_cmd_vel(const geometry_msgs::TwistConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle n;
    ros::Rate loop_rate(50); // 50Hz

    pub_motor_speed = n.advertise<std_msgs::Int16MultiArray>("/jetbot_motors/cmd_raw", 1);
    pub_feedback_motor = n.advertise<std_msgs::Int16MultiArray>("/jetbot_motors/feedback", 1);

    sub_cmd_vel = n.subscribe("/cmd_vel", 1, callback_sub_cmd_vel);

    const char *env_reverse_speed = std::getenv("REVERSE_SPEED");
    if (env_reverse_speed != NULL)
    {
        reverse_speed = atoi(env_reverse_speed);
    }
    else
    {
        reverse_speed = 0;
    }

    while (ros::ok())
    {
        callback_motor_routine();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void callback_motor_routine()
{
    is_keyboard_interrupt = (ros::Time::now().toSec() - last_time_keyboard_interrupt < 0.1);

    if (is_keyboard_interrupt)
        return;

    if (reverse_speed)
    {
        motor_speed[0] = (int16_t)(cmd_vel[1]) + cmd_vel[2] * DISTANCE_WHEEL_TO_WHEEL * 0.5;
        motor_speed[1] = (int16_t)(cmd_vel[1]) - cmd_vel[2] * DISTANCE_WHEEL_TO_WHEEL * 0.5;
        motor_speed[0] = -motor_speed[0];
        motor_speed[1] = -motor_speed[1];
    }
    else
    {
        motor_speed[0] = (int16_t)(cmd_vel[1]) - cmd_vel[2] * DISTANCE_WHEEL_TO_WHEEL * 0.5;
        motor_speed[1] = (int16_t)(cmd_vel[1]) + cmd_vel[2] * DISTANCE_WHEEL_TO_WHEEL * 0.5;
    }

    std_msgs::Int16MultiArray msg;
    msg.data.push_back(-motor_speed[0]);
    msg.data.push_back(-motor_speed[1]);
    pub_motor_speed.publish(msg);

    std_msgs::Int16MultiArray msg_feedback;
    msg_feedback.data.push_back(motor_speed[0]);
    msg_feedback.data.push_back(motor_speed[1]);
    pub_feedback_motor.publish(msg_feedback);
}

void callback_sub_cmd_vel(const geometry_msgs::TwistConstPtr &msg)
{
    cmd_vel[0] = msg->linear.x;
    cmd_vel[1] = msg->linear.y;
    cmd_vel[2] = msg->angular.z;
}