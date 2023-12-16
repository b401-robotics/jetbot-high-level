// Iki kode gawe robot pencari
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "boost/thread/mutex.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/UInt8.h"

#include "zbar.h"

#define UUID_ROBOT_1 "1111"
#define UUID_ROBOT_2 "2222"
#define UUID_ROBOT_3 "3333"

using namespace cv;
using namespace std;
using namespace zbar;

image_transport::Subscriber sub_frame_raw;
image_transport::Publisher pub_img_debugger;

ros::Publisher pub_detected_robot;

boost::mutex mutex_frame_raw;

Mat frame_raw;

std_msgs::UInt8 detected_robot;

void callback_sub_frame_raw(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_frame_raw.lock();
    frame_raw = cv_bridge::toCvCopy(msg, "bgr8")->image.clone();
    mutex_frame_raw.unlock();
}

void routine_callback()
{
    mutex_frame_raw.lock();
    Mat frame = frame_raw.clone();
    mutex_frame_raw.unlock();

    Mat frame_x_flipped;
    flip(frame, frame_x_flipped, -1);

    // Reset bit
    detected_robot.data = 0;

    // Validate frame
    if (frame_x_flipped.empty())
    {
        pub_detected_robot.publish(detected_robot);
        return;
    }

    // Using Zbar
    ImageScanner scanner;
    Mat frame_gray;
    cvtColor(frame_x_flipped, frame_gray, COLOR_BGR2GRAY);
    int width = frame_gray.cols;
    int height = frame_gray.rows;
    uchar *raw = (uchar *)(frame_gray.data);
    // wrap image data
    zbar::Image image(width, height, "Y800", raw, width * height);
    // scan the image for barcodes
    int n = scanner.scan(image);
    // extract results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        if (strcmp(symbol->get_data().c_str(), UUID_ROBOT_1) == 0)
        {
            detected_robot.data |= 0b00000001;
        }
        else if (strcmp(symbol->get_data().c_str(), UUID_ROBOT_2) == 0)
        {
            detected_robot.data |= 0b00000010;
        }
        else if (strcmp(symbol->get_data().c_str(), UUID_ROBOT_3) == 0)
        {
            detected_robot.data |= 0b00000100;
        }
    }
    // clean up
    image.set_data(NULL, 0);

    pub_detected_robot.publish(detected_robot);
    // pub_img_debugger.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_x_flipped).toImageMsg());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // 50Hz

    image_transport::ImageTransport it(n);
    sub_frame_raw = it.subscribe("/jetbot_camera/raw", 1, callback_sub_frame_raw);
    pub_img_debugger = it.advertise("/img_debugger", 1);
    pub_detected_robot = n.advertise<std_msgs::UInt8>("/detected_robot", 1);

    while (ros::ok())
    {
        routine_callback();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
