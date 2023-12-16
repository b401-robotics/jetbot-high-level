#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "boost/thread/mutex.hpp"
#include "vector"
#include "pcl/registration/icp.h"
#include "pcl/point_types.h"

#define M_PI 3.14159265358979323846
#define DEG_2_RAD 0.01745329251
#define RAD_2_DEG 57.2957795131

using namespace cv;

image_transport::Publisher pub_img_debugger;
ros::Publisher pub_d_icp;
ros::Subscriber sub_odometry;
ros::Subscriber sub_lidar_scan;

// Ukuran kotak dalam cm (dikonversi menjadi piksel)
const int scale = 5;
const int width = 148.5 * scale;
const int height = 197 * scale;

const int batas_x_min = 2;
const int batas_x_max = 142;
const int batas_y_min = 2;
const int batas_y_max = 192;

// Membuat gambar kosong
cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat map_icp(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

// ==================================================

float final_pose[3] = {0, 0, 0};
float lidar_scan[240] = {0};

std::vector<float> lidar_pcl_buff_x;
std::vector<float> lidar_pcl_buff_y;

std::vector<float> map_pcl_x;
std::vector<float> map_pcl_y;

uint16_t counter_lidar_data = 0;

// ==================================================

boost::mutex mutex_lidar_scan;

void callback_sub_odometry(const geometry_msgs::Pose2DConstPtr &msg);
void callback_sub_lidar_scan(const sensor_msgs::LaserScanConstPtr &msg);
void init_map();
void map_to_pcl();
void routine_callback();
void DrawRobot(Mat &img, float x, float y, float th, const Scalar &color);
float cm2px_x(float x);
float cm2px_y(float y);
float px2cm_x(float x);
float px2cm_y(float y);

int main(int arc, char **argv)
{
    ros::init(arc, argv, "icp");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    pub_img_debugger = it.advertise("/icp/debugger", 1);
    pub_d_icp = nh.advertise<geometry_msgs::Pose2D>("/icp/d", 1);

    sub_odometry = nh.subscribe("/odometry", 1, callback_sub_odometry);
    sub_lidar_scan = nh.subscribe("/scan", 1, callback_sub_lidar_scan);

    image = cv::Scalar(255, 255, 255);
    init_map();
    map_to_pcl();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        routine_callback();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void init_map()
{

    // Warna garis (misalnya, hitam)
    cv::Vec3b lineColor(0, 0, 0);

    // Ketebalan garis
    int thickness = 1.8 * scale;

    // Menggambar garis kotak
    cv::Point top_left(0, 0);
    cv::Point top_right(width, 0);
    cv::Point bottom_left(0, height);
    cv::Point bottom_right(width, height);

    cv::line(image, top_left, top_right, lineColor, thickness);
    cv::line(image, top_right, bottom_right, lineColor, thickness);
    cv::line(image, bottom_right, bottom_left, lineColor, thickness);
    cv::line(image, bottom_left, top_left, lineColor, thickness);

    // rintangan 4
    cv::Point start_point_4(31 * scale, (197 - 44) * scale);
    cv::Point end_point_4(31 * scale, (197 - 78.7) * scale);
    cv::Point end_point_4_1(67.1 * scale, (197 - 78.7) * scale);
    // rintangan 3
    cv::Point start_point_3(100 * scale, (197 - 40.5) * scale);
    cv::Point end_point_3(134.3 * scale, (197 - 40.5) * scale);
    cv::Point end_point_3_1(134.3 * scale, (197 - 75.2) * scale);
    // rintangan 2
    cv::Point start_point_2(132 * scale, (197 - 157.5) * scale);
    cv::Point end_point_2(132 * scale, (197 - 122.3) * scale);
    cv::Point end_point_2_1(97.5 * scale, (197 - 157.5) * scale);
    cv::Point end_point_2_2(97.5 * scale, (197 - 122.3) * scale);
    // rintangan 1
    cv::Point start_point(24 * scale, (197 - 152.5) * scale);
    cv::Point end_point(60 * scale, (197 - 152.5) * scale);
    cv::Point end_point_1(60 * scale, (197 - 118) * scale);

    // rintangan 4
    cv::line(image, start_point_4, end_point_4, lineColor, thickness);
    cv::line(image, end_point_4, end_point_4_1, lineColor, thickness);
    // rintangan 3
    cv::line(image, start_point_3, end_point_3, lineColor, thickness);
    cv::line(image, end_point_3, end_point_3_1, lineColor, thickness);
    // rintangan 2
    cv::line(image, start_point_2, end_point_2, lineColor, thickness);
    cv::line(image, start_point_2, end_point_2_1, lineColor, thickness);
    cv::line(image, end_point_2_1, end_point_2_2, lineColor, thickness);
    // rintangan 1
    cv::line(image, start_point, end_point, lineColor, thickness);
    cv::line(image, end_point, end_point_1, lineColor, thickness);
}

void map_to_pcl()
{
    // Scan every 10 pixel (5 cm) in map, if pixel is black, add to pcl
    for (int i = 0; i < image.rows; i += 10)
    {
        for (int j = 0; j < image.cols; j += 10)
        {
            if (image.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0))
            {
                map_pcl_x.push_back(j);
                map_pcl_y.push_back(i);
            }
        }
    }
}

float cm2px_x(float x)
{
    return x * scale;
}

float cm2px_y(float y)
{
    return y * scale;
}

float px2cm_x(float x)
{
    return x / scale;
}

float px2cm_y(float y)
{
    return y / scale;
}

void DrawRobot(Mat &img, float x, float y, float th, const Scalar &color)
{
    circle(img, Point(cm2px_x(x), cm2px_y(y)), 30, color, 3);
    line(img, Point(cm2px_x(x), cm2px_y(y)), Point(cm2px_x(x + 15 * cos(th * M_PI / 180)), cm2px_y(y + 15 * sin(th * M_PI / 180))), color, 3);
}

void routine_callback()
{
    mutex_lidar_scan.lock();
    if (lidar_pcl_buff_x.size() == 0)
    {
        mutex_lidar_scan.unlock();
        return;
    }
    mutex_lidar_scan.unlock();

    float pose_before_icp[3] = {final_pose[0], final_pose[1], final_pose[2]};

    // Reset image
    map_icp = cv::Scalar(255, 255, 255);

    // Create pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_sensor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_reference(new pcl::PointCloud<pcl::PointXYZ>);

    // Draw lidar scan
    mutex_lidar_scan.lock();
    for (int i = 0; i < lidar_pcl_buff_x.size(); i++)
    {
        float x = lidar_pcl_buff_x[i];
        float y = lidar_pcl_buff_y[i];

        float x_robot = x * cos((pose_before_icp[2] - 90 - 90) * DEG_2_RAD) - y * sin((pose_before_icp[2] - 90 - 90) * DEG_2_RAD);
        float y_robot = x * sin((pose_before_icp[2] - 90 - 90) * DEG_2_RAD) + y * cos((pose_before_icp[2] - 90 - 90) * DEG_2_RAD);

        x += pose_before_icp[0];
        y += pose_before_icp[1];

        x_robot += pose_before_icp[0];
        y_robot += pose_before_icp[1];

        // if (x_robot < batas_x_min || x_robot > batas_x_max || y_robot < batas_y_min || y_robot > batas_y_max)
        //     continue;

        pcl_sensor->push_back(pcl::PointXYZ(x_robot, y_robot, 0));

        circle(map_icp, Point(cm2px_x(x_robot), cm2px_y(y_robot)), 3, Scalar(0, 0, 255), -1);
    }
    mutex_lidar_scan.unlock();

    // Draw map pcl
    for (int i = 0; i < map_pcl_x.size(); i++)
    {
        pcl_reference->push_back(pcl::PointXYZ(px2cm_x(map_pcl_x[i]), px2cm_y(map_pcl_y[i]), 0));
        circle(map_icp, Point(map_pcl_x[i], map_pcl_y[i]), 3, Scalar(0, 255, 0), -1);
    }

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl_sensor);
    icp.setInputTarget(pcl_reference);
    icp.setMaximumIterations(80);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // Get ICP Score
    float icp_score = icp.getFitnessScore();

    // Get ICP transformation matrix
    Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

    float d_icp[3] = {0, 0, 0};

    /* Get Robot's translation and rotation based on ICP */
    Eigen::Vector4f point(pose_before_icp[0], pose_before_icp[1], 0, 1);
    Eigen::Vector4f transformed_point = transformation_matrix * point;
    d_icp[2] = atan2(transformation_matrix(1, 0), transformation_matrix(0, 0)) * 180 / M_PI;
    d_icp[0] = transformed_point[0] - pose_before_icp[0];
    d_icp[1] = transformed_point[1] - pose_before_icp[1];
    if (d_icp[2] > 180)
        d_icp[2] -= 360;
    else if (d_icp[2] < -180)
        d_icp[2] += 360;

    DrawRobot(map_icp, pose_before_icp[0], pose_before_icp[1], pose_before_icp[2], Scalar(0, 0, 255));
    DrawRobot(map_icp, pose_before_icp[0] + d_icp[0], pose_before_icp[1] + d_icp[1], pose_before_icp[2] + d_icp[2], Scalar(0, 255, 0));

    printf("ICP Score: %.2f -> %.2f %.2f %.2f (%d)\n", icp_score, d_icp[0], d_icp[1], d_icp[2], counter_lidar_data);

    // Flip by x-axis
    Mat image_to_pub;
    flip(map_icp, image_to_pub, 0);

    pub_img_debugger.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_to_pub).toImageMsg());

    geometry_msgs::Pose2D msg;
    msg.x = d_icp[0] * (icp_score < 62.5);
    msg.y = d_icp[1] * (icp_score < 62.5);
    msg.theta = d_icp[2] * (icp_score < 62.5);
    pub_d_icp.publish(msg);
}

void callback_sub_lidar_scan(const sensor_msgs::LaserScanConstPtr &msg)
{
    mutex_lidar_scan.lock();

    lidar_pcl_buff_x.clear();
    lidar_pcl_buff_y.clear();

    counter_lidar_data = 0;

    for (int i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] > 0 && msg->ranges[i] < 100 && msg->intensities[i] > 200)
        {
            lidar_pcl_buff_x.push_back(msg->ranges[i] * cos(i * -1 * DEG_2_RAD));
            lidar_pcl_buff_y.push_back(msg->ranges[i] * sin(i * -1 * DEG_2_RAD));
            counter_lidar_data++;
        }
    }
    mutex_lidar_scan.unlock();
}

void callback_sub_odometry(const geometry_msgs::Pose2DConstPtr &msg)
{
    final_pose[0] = msg->x;
    final_pose[1] = msg->y;
    final_pose[2] = msg->theta;
}