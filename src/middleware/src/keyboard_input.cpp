/**
 * @file keyboard_input.cpp
 * @brief Keyboard input for Jetbot
 *
 * This node is used to get keyboard input from user.
 * if key is pressed, it will publish the key value to /key topic.
 */
#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <termios.h>
#include <sys/ioctl.h>

ros::Publisher pub_keyboard_input;

static const int STDIN = 0;

void callback_keyboard_input();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_input");
    ros::NodeHandle n;
    ros::Rate loop_rate(50); // 50Hz

    pub_keyboard_input = n.advertise<std_msgs::Int8>("/key", 1);

    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);

    while (ros::ok())
    {
        callback_keyboard_input();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void callback_keyboard_input()
{
    std_msgs::Int8 msg;
    int bytesWaiting;

    if (ioctl(STDIN, FIONREAD, &bytesWaiting) == -1)
    {
        msg.data = 0;
        return;
    }

    if (bytesWaiting <= 0)
    {
        msg.data = bytesWaiting;
        return;
    }

    char c = getchar();
    msg.data = c;
    pub_keyboard_input.publish(msg);
}
