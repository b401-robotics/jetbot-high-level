/**
 * @file high_level_control.cpp
 * @brief High level control for Jetbot
 *
 * This is were the world starts.
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Pose2D.h"

#include "middleware/pid.h"
#include "middleware/simple_fsm.h"

#include "vector"

#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795131f

// State machine
#define STATE_IDLE 0x00
#define STATE_FINDING 0x01
#define STATE_WAITING_TO_BE_FOUND 0x02
#define STATE_RUNNING 0x01

// Misc
#define IS_IAM_NOT_FOUND 0b0000
#define IS_IAM_FOUND 0b0001
#define IAM_HIDE 0b0010
#define IAM_SEEK 0b0100
#define IS_IAM_CAPTAIN 0b1000

#define MAX_TIME_TO_BE_FOUND 60 // seconds
#define DEFAULT_VELOCITY 500    // pwm
#define DEFAULT_ANGULAR_VELOCITY 130

#define REGISTER_GAME_STATE 0b00001000 // bit 3
#define REGISTER_GAME_STATE_SHIFT 0x03

// #define ODOMETRY_TO_CM 0.00056862993
// #define ODOMETRY_TO_CM 0.00056329237
#define ODOMETRY_TO_CM 0.00084700033
// #define ODOMETRY_TO_CM 0.00074962518

typedef struct
{
    uint8_t current;
    uint8_t previous;
} state_t;

typedef struct
{
    int x;
    int y;
} point_t;

typedef struct
{
    float x1;
    float y1;
    float x2;
    float y2;
} zone_t;

state_t my_state;

//==============================================================================

int NOMOR_SAYA = 0b0001; // 1
int NOMOR_SAYA_SHIFT = 0x00;

//==============================================================================

double game_start_time = 0;
uint8_t who_is_hider_or_seeker = 0; // bit 0: robot 1, bit 1: robot 2, bit 2: robot 3 ||  value 1 is seeker, value 0 is hider
float final_vel[3] = {0};           // x, y, yaw
uint8_t detected_robot = 0;         // bit 0: robot 1, bit 1: robot 2, bit 2: robot 3 ||  value 1 is found, value 0 is not found
uint8_t bs_data = 0;
uint8_t bs_data_to_send = 0;
uint8_t IS_IAM_SEEK;
double last_time_get_data_from_bs = 0;

uint8_t tempat_sembunyi_1 = 0;
uint8_t tempat_sembunyi_2 = 1;
zone_t zona_lapangan[15];

const float titik_sembunyi_x[4] = {42, 114.5, 48, 117};
const float titik_sembunyi_y[4] = {61.5, 57.5, 136, 139.5};

std::vector<uint8_t> rute_menuju_target;
uint8_t zone_saya = 0;

// Odometry
float pose_buffer[3] = {0};
float final_pose[3] = {0};
float pose_offset[3] = {50, 60, 90};
int feedback_motor[2] = {0};
float feedback_vel[3] = {0};

float init_offset[3] = {0};
float d_icp[3] = {0, 0, 0};

// ICP
const float icp_gain_translation = 0.03; // 0.01 uapik iki..
const float icp_gain_rotation = 0.0f;

// MS
MachineState game_state;
MachineState ms;
MachineState hadap_yaw_berdasarkan_waktu_state;

//==============================================================================

ros::Publisher pub_cmd_vel;
ros::Publisher pub_basestation;
ros::Publisher pub_odometry;

ros::Subscriber sub_final_pose;
ros::Subscriber sub_detected_robot;
ros::Subscriber sub_basestation;
ros::Subscriber sub_yaw_position;
ros::Subscriber sub_keyboard;
ros::Subscriber sub_feedback_motor;
ros::Subscriber sub_d_icp;

/**
 * @brief Ini adalah callback yang akan selalu dijalankan setiap 50 Hz
 */
void callback_routine();

/**
 * DEPRECATED !!!
 * @brief Ini adalah callback untuk Menerima Pose dari Pose estimator
 */
void callback_sub_final_pose(const geometry_msgs::Pose2DConstPtr &msg);

/**
 * @brief Ini adalah callback untuk menerima data dari QR Detector
 *
 * Bit selection, bit 0 untuk robot 1, bit 1 untuk robot 2, dan bit 2 untuk robot 3.
 *
 * @example 0b100 manandakan deteksi robot 3
 */
void callback_sub_detected_robot(const std_msgs::UInt8ConstPtr &msg);

/**
 * @brief Ini adalah callback untuk menerima
 * 0 -> robot 1
 * 1 -> robot 2
 * 2 -> robot 3
 * 3 -> robot game state (start / stop)
 * 4 5 -> Tempat sembunyi 1
 * 6 7 -> Tempat sembunyi 2
 */
void callback_sub_basestation(const std_msgs::UInt8ConstPtr &msg);

/**
 * @brief Ini adalah callback untuk menerima data dari IMU berupa gyro yaw
 */
void callback_sub_yaw_position(const std_msgs::Float64ConstPtr &msg);

/**
 * @brief Ini adalah callback untuk menerima data dari keyboard
 */
void callback_sub_keyboard(const std_msgs::Int8ConstPtr &msg);

/**
 * DEPRECATED !!!
 * @brief Ini adalah callback untuk menerima data dari motor
 */
void callback_sub_feedback_motor(const std_msgs::Int16MultiArrayConstPtr &msg);

/**
 * @brief Ini adalah callback untuk menerima data dari ICP
 *
 * Data yang diterima adalah delta ICP estimasi terhadap pose robot
 */
void callback_sub_d_icp(const geometry_msgs::Pose2DConstPtr &msg);

/**
 * @brief Ini adalah fungsi sub algoritma untuk mencari lawan
 *
 * Disini menunakan hardcode untuk mencari lawan
 */
void jalan_bebas_dan_temukan();

/**
 * @brief Ini adalah fungsi sub algoritma untuk mencari tempat bersembunyi
 *
 * Tempat bersembunyi ditentukan oleh basestation
 */
void cari_tempat_bersembunyi();

/**
 * @brief Ini adalah fungsi sub algoritma untuk mencari tempat bersembunyi
 *
 * @param x adalah velocity x  (harus 0)
 * @param y adalah velocity y  (kecepatan maju robot)
 * @param yaw adalah velocity yaw (kecepatan rotasi robot)
 */
void set_velocity(float x, float y, float yaw);

/**
 * DEPRECATED !!!
 * @brief Ini adalha fungsi untuk merubah kecepatan robot berbasis global menjadi berbasis local
 *
 * @param x adalah kecepatan x global
 * @param y adalah kecepatan y global
 * @param yaw adalah yaw global
 */
void global_2_local_velocity(float x, float y, float yaw);

/**
 * DEPRECATED !!!
 * @brief Ini adalah fungsi untuk menggerakkan robot maju lurus
 *
 * @param vy adalah kecepatan maju robot
 * @param target_yaw adalah yaw yang diinginkan
 */
void gerak_maju_lurus(float vy, float target_yaw);

/**
 * @brief Ini adalah fungsi untuk menghadapkan robot ke arah yaw yang diinginkan
 *
 * Fungsi ini menerapkan satu sistem kontrol PID dengan kecepatan rotasi sebagai output dan gyro yaw sebagai feedback
 *
 * @param yaw adalah yaw yang diinginkan
 * @return 1 jika sudah menghadap, 0 jika belum
 */
int8_t hadap_yaw(float yaw);

/**
 * DEPRECATED !!!
 * @brief Ini adalah fungsi untuk menghadapkan robot ke arah yaw yang diinginkan
 *
 * Fungsi ini open loop sistem kontrol dengan kecepatan rotasi sebagai output
 *
 * @param yaw adalah yaw yang diinginkan
 * @return 1 jika sudah menghadap, 0 jika belum
 */
int8_t hadap_yaw_berdasarkan_waktu(float yaw);

/**
 * @brief Fungsi ini digunakan untuk menggerakkan robot menuju satu titik tertentu
 *
 * Fungsi ini menerapkan dua sistem kontrol PID, yaitu translasi dan rotasi
 *
 * @param x adalah posisi x tujuan
 * @param y adalah posisi y tujuan
 * @param yaw adalah yaw tujuan (Ini sebenarnya tidak digunakan karena default nya arah robot adalah arah menuju titik)
 * @return 1 jika sudah mencapai titik, 0 jika belum
 */
int8_t motion_to_point(float x, float y, float yaw);

/**
 * DEPRECATED !!!
 * @brief Fungsi ini digunakan untuk menggerakkan robot menuju satu titik tertentu
 *
 * Fungsi ini menerapkan dua sistem kontrol PID, yaitu translasi dan rotasi
 *
 * @param x adalah posisi x tujuan
 * @param y adalah posisi y tujuan
 * @param yaw adalah yaw tujuan (Ini sebenarnya tidak digunakan karena default nya arah robot adalah arah menuju titik)
 * @return 1 jika sudah mencapai titik, 0 jika belum
 */
int8_t motion_to_point_2(float x, float y, float yaw);

/**
 * @brief Fungsi ini digunakan untuk mengolah odometry robot dengan menggunakan kecepatan output dari set_velocity() dan
 * gyro yaw sebagai IMU
 */
void odometry();

/**
 * @brief Fungsi ini digunakan untuk mengecek apakah semua robot sudah ditemukan
 *
 * Data disimpan langsung pada bs_data_to_send
 *
 * @return 1 jika semua robot sudah ditemukan, 0 jika belum
 */
uint8_t check_all_robot_is_found();

/**
 * @brief Fungsi ini digunakan untuk menginisisasi zona lapangan
 */
void init_zona_lapangan();

/**
 * @brief Fungsi ini digunakan untuk membuat zona lapangan
 */
void set_zona_lapangan(int index, float x1, float y1, float x2, float y2);

/**
 * @brief Fungsi ini digunakan untuk mengecek robot berada di zona mana
 *
 * @return zona yang dideteksi
 */
uint8_t check_robot_on_zone();

/**
 * @brief Fungsi ini digunakan untuk mendapatkan centroid dari zona,
 *
 * Ada beberapa hardocde untuk zona tertentu
 *
 * @return centroid dari zona
 */
point_t get_centroid_of_zone(uint8_t zone);

/**
 * @brief Fungsi ini digunakan untuk mengolah FSM dari game
 *
 * FSM adalah Finite State Machine
 */
void game();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "high_level_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(50); // 50Hz

    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_basestation = n.advertise<std_msgs::UInt8>("/basestation_tx", 1);
    pub_odometry = n.advertise<geometry_msgs::Pose2D>("/odometry", 1);

    // sub_final_pose = n.subscribe("/final_pose", 1, callback_sub_final_pose);
    sub_detected_robot = n.subscribe("/detected_robot", 1, callback_sub_detected_robot);
    sub_basestation = n.subscribe("/basestation_rx", 1, callback_sub_basestation);
    sub_yaw_position = n.subscribe("/yaw_position", 1, callback_sub_yaw_position);
    sub_keyboard = n.subscribe("/key", 1, callback_sub_keyboard);
    sub_feedback_motor = n.subscribe("/jetbot_motors/feedback", 1, callback_sub_feedback_motor);
    sub_d_icp = n.subscribe("/icp/d", 1, callback_sub_d_icp);

    const char *robot_num_str = std::getenv("ROBOT_NUM"); // Get robot number from environment variable ROBOT_NUM

    int robot_num = std::atoi(robot_num_str);

    switch (robot_num)
    {
    case 1:
        NOMOR_SAYA = 0b0001;
        NOMOR_SAYA_SHIFT = 0x00;

        pose_offset[0] = 40;
        pose_offset[1] = 177;
        pose_offset[2] = 0;

        break;
    case 2:
        NOMOR_SAYA = 0b0010;
        NOMOR_SAYA_SHIFT = 0x01;

        pose_offset[0] = 30;
        pose_offset[1] = 30;
        pose_offset[2] = 0;
        break;
    case 3:
        NOMOR_SAYA = 0b0100;
        NOMOR_SAYA_SHIFT = 0x02;

        pose_offset[0] = 50;
        pose_offset[1] = 60;
        pose_offset[2] = 90;
        break;
    default:
        NOMOR_SAYA = 0b0001;
        NOMOR_SAYA_SHIFT = 0x00;
        break;
    }

    init_offset[0] = pose_offset[0];
    init_offset[1] = pose_offset[1];
    init_offset[2] = pose_offset[2];

    init_zona_lapangan();

    while (ros::ok())
    {
        callback_routine();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

//==============================================================================

point_t get_centroid_of_zone(uint8_t zone)
{
    point_t centroid;

    if (zone == 13 && zone_saya == 12)
    {
        centroid.x = (zona_lapangan[zone].x1 + zona_lapangan[zone].x2) / 2 + 1;
        centroid.y = (zona_lapangan[zone].y1 + zona_lapangan[zone].y2) / 2 - 20;

        return centroid;
    }
    else if (zone == 7 && zone_saya == 12)
    {
        centroid.x = (zona_lapangan[zone].x1 + zona_lapangan[zone].x2) / 2;
        centroid.y = (zona_lapangan[zone].y1 + zona_lapangan[zone].y2) / 2;

        return centroid;
    }
    else if (zone == 6 && zone_saya == 12)
    {
        centroid.x = (zona_lapangan[zone].x1 + zona_lapangan[zone].x2) / 2 - 10;
        centroid.y = (zona_lapangan[zone].y1 + zona_lapangan[zone].y2) / 2 - 1;

        return centroid;
    }
    else if (zone == 6 && zone_saya == 12)
    {
        centroid.x = (zona_lapangan[zone].x1 + zona_lapangan[zone].x2) / 2 - 3;
        centroid.y = (zona_lapangan[zone].y1 + zona_lapangan[zone].y2) / 2 - 1;

        return centroid;
    }
    else if (zone == 9)
    {
        centroid.x = titik_sembunyi_x[2];
        centroid.y = titik_sembunyi_y[2];

        return centroid;
    }
    else if (zone == 11)
    {
        centroid.x = titik_sembunyi_x[3];
        centroid.y = titik_sembunyi_y[3];

        return centroid;
    }
    else if (zone == 3)
    {
        centroid.x = titik_sembunyi_x[0];
        centroid.y = titik_sembunyi_y[0];

        return centroid;
    }
    else if (zone == 5)
    {
        centroid.x = titik_sembunyi_x[1] - 7;
        centroid.y = titik_sembunyi_y[1];

        return centroid;
    }

    centroid.x = (zona_lapangan[zone].x1 + zona_lapangan[zone].x2) / 2;
    centroid.y = (zona_lapangan[zone].y1 + zona_lapangan[zone].y2) / 2;

    return centroid;
}

uint8_t check_robot_on_zone()
{
    for (int i = 0; i < 15; i++)
    {
        if (final_pose[0] > zona_lapangan[i].x1 && final_pose[0] < zona_lapangan[i].x2 && final_pose[1] > zona_lapangan[i].y1 && final_pose[1] < zona_lapangan[i].y2)
            return i;
    }

    return 0;
}

void set_zona_lapangan(int index, float x1, float y1, float x2, float y2)
{
    zona_lapangan[index].x1 = x1;
    zona_lapangan[index].y1 = y1;
    zona_lapangan[index].x2 = x2;
    zona_lapangan[index].y2 = y2;
}

void init_zona_lapangan()
{
    set_zona_lapangan(0, 0, 0, 62, 39.4);
    set_zona_lapangan(1, 62, 0, 99, 39.4);
    set_zona_lapangan(2, 99, 0, 148.5, 39.4);

    set_zona_lapangan(3, 0, 39.4, 62, 78.8);
    set_zona_lapangan(4, 62, 39.4, 99, 78.8);
    set_zona_lapangan(5, 99, 39.4, 148.5, 78.8);

    set_zona_lapangan(6, 0, 78.8, 62, 118.2);
    set_zona_lapangan(7, 62, 78.8, 99, 118.2);
    set_zona_lapangan(8, 99, 78.8, 148.5, 118.2);

    set_zona_lapangan(9, 0, 118.2, 62, 157.6);
    set_zona_lapangan(10, 62, 118.2, 99, 157.6);
    set_zona_lapangan(11, 99, 118.2, 148.5, 157.6);

    set_zona_lapangan(12, 0, 157.6, 62, 197);
    set_zona_lapangan(13, 62, 157.6, 99, 197);
    set_zona_lapangan(14, 99, 157.6, 148.5, 197);
}

void jalan_bebas_dan_temukan()
{
    ms.reentry(0, 1);
    printf("Mencari lawan state: %d\n", ms.value);

    point_t centroid_zone;
    float sudut_robot_ke_titik_sembunyi = 0;

    switch (ms.value)
    {
    case 0:
        rute_menuju_target.clear();
        ms.value = 1;
        break;

    case 1:
        zone_saya = check_robot_on_zone();

        if (zone_saya < 6)
            rute_menuju_target.push_back(1);
        else if (zone_saya >= 9)
            rute_menuju_target.push_back(13);
        rute_menuju_target.push_back(7);
        rute_menuju_target.push_back(6);
        ms.value = 2;
        break;

    case 2:
        if (rute_menuju_target.size() > 0)
        {
            centroid_zone = get_centroid_of_zone(rute_menuju_target[0]);
            // Direction is pose to the centroid

            printf("Target to move: %d %d %.2f (%d)\n", centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG, rute_menuju_target[0]);
            if (motion_to_point(centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG))
                rute_menuju_target.erase(rute_menuju_target.begin());
        }
        else
        {
            ms.value = 3;
        }
        break;
    case 3:
        sudut_robot_ke_titik_sembunyi = atan2f(titik_sembunyi_y[0] - final_pose[1], titik_sembunyi_x[0] - final_pose[0]) * RAD2DEG;
        if (hadap_yaw(sudut_robot_ke_titik_sembunyi))
        {
            ms.value = 40;
            ms.resetUptimeTimeout();
        }
        break;
    case 40:
        set_velocity(0, 0, 0);
        ms.timeout(4, 2);
        break;
    case 4:
        rute_menuju_target.push_back(7);
        rute_menuju_target.push_back(8);
        ms.value = 50;
        break;
    case 50:
        if (rute_menuju_target.size() > 0)
        {
            centroid_zone = get_centroid_of_zone(rute_menuju_target[0]);
            // Direction is pose to the centroid

            printf("Target to move: %d %d %.2f (%d)\n", centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG, rute_menuju_target[0]);
            if (motion_to_point(centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG))
                rute_menuju_target.erase(rute_menuju_target.begin());
        }
        else
        {
            ms.value = 5;
        }
        break;
    case 5:
        sudut_robot_ke_titik_sembunyi = atan2f(titik_sembunyi_y[1] - final_pose[1], titik_sembunyi_x[1] - final_pose[0]) * RAD2DEG;
        if (hadap_yaw(sudut_robot_ke_titik_sembunyi))
        {
            ms.value = 60;
            ms.resetUptimeTimeout();
        }
        break;
    case 60:
        set_velocity(0, 0, 0);
        ms.timeout(6, 2);
        break;
    case 6:
        sudut_robot_ke_titik_sembunyi = atan2f(titik_sembunyi_y[3] - final_pose[1], titik_sembunyi_x[3] - final_pose[0]) * RAD2DEG;
        if (hadap_yaw(sudut_robot_ke_titik_sembunyi))
        {
            ms.value = 70;
            ms.resetUptimeTimeout();
        }
        break;
    case 70:
        set_velocity(0, 0, 0);
        ms.timeout(7, 2);
        break;
    case 7:
        rute_menuju_target.push_back(10);
        ms.value = 8;
        break;
    case 8:
        if (rute_menuju_target.size() > 0)
        {
            centroid_zone = get_centroid_of_zone(rute_menuju_target[0]);
            printf("Target to move: %d %d %.2f (%d)\n", centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG, rute_menuju_target[0]);
            if (motion_to_point(centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG))
                rute_menuju_target.erase(rute_menuju_target.begin());
        }
        else
        {
            ms.value = 9;
        }
        break;
    case 9:
        sudut_robot_ke_titik_sembunyi = atan2f(titik_sembunyi_y[2] - final_pose[1], titik_sembunyi_x[2] - final_pose[0]) * RAD2DEG;
        if (hadap_yaw(sudut_robot_ke_titik_sembunyi))
        {
            ms.value = 100;
            ms.resetUptimeTimeout();
        }
        break;
    case 100:
        set_velocity(0, 0, 0);
        ms.timeout(1, 2);
        break;
    }
}

void cari_tempat_bersembunyi()
{
    uint8_t nomor_seeker = bs_data & 0b111;
    static uint8_t target_sembunyi = 0;

    point_t centroid_zone;

    ms.reentry(0, 1);

    printf("Cari tempat bersembunyi state: %d\n", ms.value);

    switch (ms.value)
    {
    case 0:
        rute_menuju_target.clear();

        if (NOMOR_SAYA == 0b001)
        {
            target_sembunyi = tempat_sembunyi_1;
        }
        else if (NOMOR_SAYA == 0b010)
        {
            if (nomor_seeker == 0b001)
                target_sembunyi = tempat_sembunyi_1;
            else if (nomor_seeker == 0b100)
                target_sembunyi = tempat_sembunyi_2;
        }
        else if (NOMOR_SAYA == 0b100)
        {
            target_sembunyi = tempat_sembunyi_2;
        }

        ms.value = 1;
        break;

    case 1:
        zone_saya = check_robot_on_zone();

        if (target_sembunyi == 0 && zone_saya < 3)
        {
            rute_menuju_target.push_back(1);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(6);
            rute_menuju_target.push_back(3);
        }
        else if (target_sembunyi == 0 && zone_saya >= 9)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(6);
            rute_menuju_target.push_back(3);
        }
        else if (target_sembunyi == 0 && zone_saya == 14)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(6);
            rute_menuju_target.push_back(3);
        }
        else if (target_sembunyi == 1 && zone_saya < 3)
        {
            rute_menuju_target.push_back(1);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(8);
            rute_menuju_target.push_back(5);
        }
        else if (target_sembunyi == 1 && zone_saya >= 9)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(8);
            rute_menuju_target.push_back(5);
        }
        else if (target_sembunyi == 1 && zone_saya == 14)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(8);
            rute_menuju_target.push_back(5);
        }
        else if (target_sembunyi == 2 && zone_saya < 3)
        {
            rute_menuju_target.push_back(1);
            rute_menuju_target.push_back(10);
            rute_menuju_target.push_back(9);
        }
        else if (target_sembunyi == 2 && zone_saya >= 9)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(9);
        }
        else if (target_sembunyi == 2 && zone_saya == 14)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(9);
        }
        else if (target_sembunyi == 3 && zone_saya < 3)
        {
            rute_menuju_target.push_back(1);
            rute_menuju_target.push_back(7);
            rute_menuju_target.push_back(11);
        }
        else if (target_sembunyi == 3 && zone_saya >= 9)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(10);
            rute_menuju_target.push_back(11);
        }
        else if (target_sembunyi == 3 && zone_saya == 14)
        {
            rute_menuju_target.push_back(13);
            rute_menuju_target.push_back(10);
            rute_menuju_target.push_back(11);
        }

        ROS_ERROR("%d %d", target_sembunyi, zone_saya);

        ms.value = 2;
        break;

    case 2:
        if (rute_menuju_target.size() > 0)
        {
            centroid_zone = get_centroid_of_zone(rute_menuju_target[0]);
            // Direction is pose to the centroid

            printf("Target to move: %d %d %.2f (%d)\n", centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG, rute_menuju_target[0]);
            if (motion_to_point(centroid_zone.x, centroid_zone.y, atan2(centroid_zone.y - final_pose[1], centroid_zone.x - final_pose[0]) * RAD2DEG))
                rute_menuju_target.erase(rute_menuju_target.begin());
        }
        else
        {
            ms.value = 3;
        }
        break;

    case 3:
        set_velocity(0, 0, 0);
        break;
    }
}

int8_t motion_to_point(float x, float y, float yaw)
{
    static PID_t pid_translasi;
    static PID_t pid_rotasi;

    PIDInit(&pid_translasi, 20, 0.1, 0.1);
    PIDInit(&pid_rotasi, 3, 0, 0.1);

    float error_x = x - final_pose[0];
    float error_y = y - final_pose[1];
    float error_distance = sqrtf(error_x * error_x + error_y * error_y);
    float angle_to_point = atan2f(error_y, error_x) * RAD2DEG;

    float error_yaw = angle_to_point - final_pose[2];

    while (error_yaw > 180)
        error_yaw -= 360;
    while (error_yaw < -180)
        error_yaw += 360;

    while (angle_to_point > 180)
        angle_to_point -= 360;
    while (angle_to_point < -180)
        angle_to_point += 360;

    float out_translasi = PIDCalculate(&pid_translasi, error_distance, DEFAULT_VELOCITY);
    float out_rotasi = PIDCalculate(&pid_rotasi, error_yaw, DEFAULT_ANGULAR_VELOCITY);

    float out_x_global = out_translasi * cosf(angle_to_point * DEG2RAD);
    float out_y_global = out_translasi * sinf(angle_to_point * DEG2RAD);

    float out_x_local = out_x_global * sinf(final_pose[2] * DEG2RAD) - out_y_global * cosf(final_pose[2] * DEG2RAD);
    float out_y_local = out_x_global * cosf(final_pose[2] * DEG2RAD) + out_y_global * sinf(final_pose[2] * DEG2RAD);

    printf("error: %.2f %.2f out: %.2f %.2f (%.2f)\n", error_distance, error_yaw, out_y_local, out_rotasi, angle_to_point);

    set_velocity(0, out_y_local, out_rotasi);

    if (error_distance < 13 && fabsf(error_yaw) < 12)
        return 1;
    else
        return 0;
}

int8_t hadap_yaw_berdasarkan_waktu(float yaw)
{
    static double time_start = 0;
    static float out_vel = 0;
    static float time_to_finish = 0;

    hadap_yaw_berdasarkan_waktu_state.reentry(0, 1);

    printf("state: %d %.2f %.2f %.2f %.2f\n", hadap_yaw_berdasarkan_waktu_state.value, yaw, out_vel, fabsf(yaw - final_pose[2]), time_to_finish);

    switch (hadap_yaw_berdasarkan_waktu_state.value)
    {
    case 0:
        time_start = ros::Time::now().toSec();

        out_vel = yaw - final_pose[2];

        while (out_vel > 180)
            out_vel -= 360;
        while (out_vel < -180)
            out_vel += 360;

        time_to_finish = fabsf(out_vel) / DEFAULT_ANGULAR_VELOCITY;

        if (out_vel > 0)
            out_vel = DEFAULT_ANGULAR_VELOCITY;
        else if (out_vel < 0)
            out_vel = -DEFAULT_ANGULAR_VELOCITY;

        hadap_yaw_berdasarkan_waktu_state.value = 1;
        break;
    case 1:
        set_velocity(0, 0, out_vel);
        if (ros::Time::now().toSec() - time_start > time_to_finish)
            hadap_yaw_berdasarkan_waktu_state.value = 2;
        break;
    case 2:
        set_velocity(0, 0, 0);
        hadap_yaw_berdasarkan_waktu_state.value = 0;
        break;
    }

    if (hadap_yaw_berdasarkan_waktu_state.value == 2)
        return 1;
    else
        return 0;
}

int8_t motion_to_point_2(float x, float y, float yaw)
{
    static MachineState motion_state;
    motion_state.reentry(0, 1);

    static float target_yaw = 0;
    float error_distance;

    float error_x = x - final_pose[0];
    float error_y = y - final_pose[1];
    error_distance = sqrtf(error_x * error_x + error_y * error_y);

    float one_element_to_target = 99999;

    printf("Motion state: %d -> %.2f %.2f %.2f (%.2f)\n", motion_state.value, error_x, error_y, error_distance, one_element_to_target);

    switch (motion_state.value)
    {
    case 0:
        target_yaw = atan2f(y - final_pose[1], x - final_pose[0]) * RAD2DEG;

        if (hadap_yaw(target_yaw))
        {
            printf("=============================================== %.2f (%.2f %.2f %.2f %.2f)\n", target_yaw, x, y, final_pose[0], final_pose[1]);
            motion_state.value = 1;
        }

        break;

    case 1:
        target_yaw = atan2f(y - final_pose[1], x - final_pose[0]) * RAD2DEG;

        gerak_maju_lurus(DEFAULT_VELOCITY, target_yaw);

        error_distance = sqrtf((x - final_pose[0]) * (x - final_pose[0]) + (y - final_pose[1]) * (y - final_pose[1]));

        if (fabsf(sinf(target_yaw * DEG2RAD)) > fabsf(cosf(target_yaw * DEG2RAD)))
            one_element_to_target = error_distance * sinf(target_yaw * DEG2RAD);
        else
            one_element_to_target = error_distance * cosf(target_yaw * DEG2RAD);

        printf("QQEQEQWEQWEWQ %.2f (%.2f)\n", error_distance * cosf(target_yaw * DEG2RAD), target_yaw);

        if (error_distance < 10)
            motion_state.value = 3;
        break;

    case 2:
        target_yaw = yaw;
        gerak_maju_lurus(0, target_yaw);
        if (fabsf(target_yaw - final_pose[2]) < 5)
            motion_state.value = 3;
        break;

    case 3:
        motion_state.value = 0;
        break;
    }

    if (motion_state.value == 3)
        return 1;
    else
        return 0;
}

void odometry()
{
    static float feedback_vel_local[3] = {0};

    feedback_vel_local[0] = 0;
    feedback_vel_local[1] = feedback_motor[0] + feedback_motor[1];

    pose_buffer[0] += final_vel[1] * cosf(final_pose[2] * DEG2RAD) * ODOMETRY_TO_CM;
    pose_buffer[1] += final_vel[1] * sinf(final_pose[2] * DEG2RAD) * ODOMETRY_TO_CM;

    // if (d_icp[0] != 0 && d_icp[1] != 0 && d_icp[2] != 0)
    // {
    //     float target_robot_offset[3] = {0};
    //     target_robot_offset[0] = (1 - icp_gain_translation) * (pose_offset[0] + pose_buffer[0]) + icp_gain_translation * ((pose_offset[0] + pose_buffer[0]) + d_icp[0]);
    //     target_robot_offset[1] = (1 - icp_gain_translation) * pose_offset[1] + pose_buffer[1] + icp_gain_translation * (pose_offset[1] + pose_buffer[1] + d_icp[1]);
    //     target_robot_offset[2] = (1 - icp_gain_rotation) * final_pose[2] + icp_gain_rotation * (final_pose[2] + d_icp[2]);

    //     // printf("%.2f %.2f %.2f || %.2f %.2f %.2f\n", final_pose[0], final_pose[1], final_pose[2],
    //     //        target_robot_offset[0], target_robot_offset[1], target_robot_offset[2]);

    //     pose_offset[0] = target_robot_offset[0] - pose_buffer[0];
    //     pose_offset[1] = target_robot_offset[1] - pose_buffer[1];
    //     // pose_offset[2] = target_robot_offset[2] - pose_buffer[2];
    // }

    final_pose[0] = pose_offset[0] + pose_buffer[0];
    final_pose[1] = pose_offset[1] + pose_buffer[1];
    final_pose[2] = pose_offset[2] + pose_buffer[2];
}

int8_t hadap_yaw(float yaw)
{
    static PID_t pid_sudut;

    PIDInit(&pid_sudut, 3.1, 0.05, 0.1);

    float error_yaw = yaw - final_pose[2];

    while (error_yaw > 180)
        error_yaw -= 360;
    while (error_yaw < -180)
        error_yaw += 360;

    float out_sudut = PIDCalculate(&pid_sudut, error_yaw, DEFAULT_ANGULAR_VELOCITY);

    set_velocity(0, 0, out_sudut);

    printf("ERROR HADAP %.2f || OUT %.2f\n", fabs(error_yaw), out_sudut);

    if (fabsf(error_yaw) < 7)
        return 1;
    else
        return 0;
}

void gerak_maju_lurus(float vy, float target_yaw)
{
    static PID_t pid_sudut;

    PIDInit(&pid_sudut, 7, 0, 0);

    float error_yaw = target_yaw - final_pose[2];

    while (error_yaw > 180)
        error_yaw -= 360;
    while (error_yaw < -180)
        error_yaw += 360;

    float out_sudut = PIDCalculate(&pid_sudut, error_yaw, DEFAULT_ANGULAR_VELOCITY * 0.5);

    printf("error: %.2f out: %.2f\n", error_yaw, out_sudut);

    set_velocity(0, vy, out_sudut);
}

void global_2_local_velocity(float x, float y, float yaw)
{
    float buffer[3] = {0};
    float yaw_rad = yaw * DEG2RAD;
    buffer[0] = x * cosf(yaw_rad) + y * sinf(yaw_rad);
    buffer[1] = -x * sinf(yaw_rad) + y * cosf(yaw_rad);
    buffer[2] = yaw;

    set_velocity(buffer[0], buffer[1], buffer[2]);
}

void set_velocity(float x, float y, float yaw)
{
    final_vel[0] = x;
    final_vel[1] = y;
    final_vel[2] = yaw;
}

void game()
{
    game_state.reentry(0, 1);

    printf("Game state: %d\n", game_state.value);

    switch (game_state.value)
    {
    case 0:
        ms.value = 0;
        bs_data_to_send = 0;
        if (IS_IAM_SEEK)
            game_state.value = 10;
        else
            game_state.value = 2;
        game_state.resetUptimeTimeout();
        break;

    case 10:
        set_velocity(0, 0, 0);
        game_state.timeout(1, 10);
        break;

    case 1:
        jalan_bebas_dan_temukan();

        check_all_robot_is_found();

        break;

    case 2:
        cari_tempat_bersembunyi();
        break;
    }
}

uint8_t check_all_robot_is_found()
{
    static const uint8_t base = 0b00000111;
    static uint8_t buffer = base & ~NOMOR_SAYA;
    static uint8_t detected_robot_global_buffer = 0;
    detected_robot_global_buffer = 0;
    detected_robot_global_buffer |= 0b100;

    buffer = base & ~NOMOR_SAYA;
    buffer &= ~(bs_data & 0b111);

    my_state.current == 10 ? detected_robot_global_buffer |= detected_robot : detected_robot_global_buffer = 0;

    printf("Detect: %d %d\n", detected_robot_global_buffer, buffer);

    bs_data_to_send = (detected_robot_global_buffer == buffer);

    return (detected_robot_global_buffer == buffer);
}

//==============================================================================

void callback_routine()
{
    odometry();

    if (ros::Time::now().toSec() - last_time_get_data_from_bs < 1) // Tolerenasi keterlamabatan data dari basestation 1 detik
    {
        my_state.current = 10 * ((bs_data & REGISTER_GAME_STATE) >> REGISTER_GAME_STATE_SHIFT);
        IS_IAM_SEEK = (bs_data & NOMOR_SAYA) >> NOMOR_SAYA_SHIFT;

        tempat_sembunyi_1 = (bs_data & 0b00110000) >> 4;
        tempat_sembunyi_2 = (bs_data & 0b11000000) >> 6;
    }

    if (my_state.previous == 10 && my_state.current == 0)
        ROS_WARN("GAME FINISHED");

    switch (my_state.current)
    {
    case 0:
        // Idle
        bs_data_to_send = 0;
        game_state.value = 0;
        ms.value = 0;
        set_velocity(0, 0, 0);
        break;
    case 1:
        set_velocity(0, DEFAULT_VELOCITY, 0);
        break;
    case 2:
        set_velocity(0, -DEFAULT_VELOCITY, 0);
        break;
    case 3:
        set_velocity(0, 0, -DEFAULT_ANGULAR_VELOCITY);
        break;
    case 4:
        set_velocity(0, 0, DEFAULT_ANGULAR_VELOCITY);
        break;
    case 5:
        gerak_maju_lurus(DEFAULT_VELOCITY, 0);
        break;
    case 6:
        motion_to_point_2(50, 0, 45);
        break;
    case 10:
        game();
        break;

    case 90:
        if (hadap_yaw(-90))
        {
            ROS_WARN("DONE");
        }
        break;

    case 99:
        if (hadap_yaw(90))
        {
            ROS_WARN("DONE");
        }
        break;

    case 100:
        cari_tempat_bersembunyi();
        break;

    case 101:
        if (motion_to_point_2(100, 100, 90))
            my_state.current = 102;
        break;

    case 102:
        if (motion_to_point_2(0, 100, 180))
            my_state.current = 103;
        break;

    case 103:
        if (motion_to_point_2(0, 0, 270))
            my_state.current = 100;
        break;
    }

    my_state.previous = my_state.current;

    geometry_msgs::Twist msg;
    msg.linear.x = final_vel[0];
    msg.linear.y = final_vel[1];
    msg.angular.z = final_vel[2];
    pub_cmd_vel.publish(msg);

    std_msgs::UInt8 msg_bs;
    msg_bs.data = bs_data_to_send;
    pub_basestation.publish(msg_bs);

    geometry_msgs::Pose2D msg_odometry;
    msg_odometry.x = final_pose[0];
    msg_odometry.y = final_pose[1];
    msg_odometry.theta = final_pose[2];
    pub_odometry.publish(msg_odometry);
}

void callback_sub_feedback_motor(const std_msgs::Int16MultiArrayConstPtr &msg)
{
    feedback_motor[0] = msg->data[0];
    feedback_motor[1] = msg->data[1];
}

void callback_sub_yaw_position(const std_msgs::Float64ConstPtr &msg)
{
    pose_buffer[2] = msg->data;
}

void callback_sub_basestation(const std_msgs::UInt8ConstPtr &msg)
{
    last_time_get_data_from_bs = ros::Time::now().toSec();
    bs_data = msg->data;
    // Bit 0: robot 1, bit 1: robot 2, bit 2: robot 3 || value 1 is hide, value 0 is seek
    // Bit 3: start game || value 1 is start, value 0 is stop
}

void callback_sub_final_pose(const geometry_msgs::Pose2DConstPtr &msg)
{
    final_pose[0] = msg->x;
    final_pose[1] = msg->y;
    final_pose[2] = msg->theta;
}

void callback_sub_keyboard(const std_msgs::Int8ConstPtr &msg)
{
    if (msg->data > 0)
    {
        switch (msg->data)
        {
        case 'j':
            my_state.current = 1;
            break;
        case 'n':
            my_state.current = 2;
            break;
        case 'm':
            my_state.current = 3;
            break;
        case 'b':
            my_state.current = 4;
            break;
        case ' ':
            my_state.current = 0;
            break;
        case '2':
            my_state.current = 5;
            break;
        case '3':
            my_state.current = 6;
            break;

        case '4':
            my_state.current = 100;
            break;

        case '9':
            my_state.current = 99;
            break;

        case '0':
            my_state.current = 90;
            break;

        case 'o':
            pose_offset[0] = init_offset[0] - pose_buffer[0];
            pose_offset[1] = init_offset[1] - pose_buffer[1];
            pose_offset[2] = init_offset[2] - pose_buffer[2];
            break;
        }
    }
}

void callback_sub_d_icp(const geometry_msgs::Pose2DConstPtr &msg)
{
    d_icp[0] = msg->x;
    d_icp[1] = msg->y;
    d_icp[2] = msg->theta;
    if (fabs(msg->x) > 75 || fabs(msg->y) > 75 || fabs(msg->theta) > 60)
    {
        ROS_WARN("ICP FALSE DETECT");
        return;
    }
    if (msg->x != 0 && msg->y != 0 && msg->theta != 0)
    {
        float target_robot_offset[3] = {0};
        target_robot_offset[0] = (1 - icp_gain_translation) * final_pose[0] + icp_gain_translation * (final_pose[0] + msg->x);
        target_robot_offset[1] = (1 - icp_gain_translation) * final_pose[1] + icp_gain_translation * (final_pose[1] + msg->y);
        target_robot_offset[2] = (1 - icp_gain_rotation) * final_pose[2] + icp_gain_rotation * (final_pose[2] + msg->theta);

        pose_offset[0] = target_robot_offset[0] - pose_buffer[0];
        pose_offset[1] = target_robot_offset[1] - pose_buffer[1];
        // pose_offset[2] = target_robot_offset[2] - pose_buffer[2];
    }
}

void callback_sub_detected_robot(const std_msgs::UInt8ConstPtr &msg)
{
    detected_robot = msg->data;
}