#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <geometry_msgs/Twist.h>

#define MPU6050_I2C_ADDR 0x68

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_FIFO_EN 0x23
#define REG_USER_CTRL 0x6A
#define REG_FIFO_COUNT_L 0x72
#define REG_FIFO_COUNT_H 0x73
#define REG_FIFO 0x74
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E

float accel_offset;
float accel_offset_buffer = 0;
float accel_array[1000];
float accel_standart_deviation = 0;

float gyro_offset = 0.0f;
float gyro_offset_buffer = 0.0f;

int file = -1;

void i2c_write(__u8 reg_address, __u8 val)
{
    char buf[2];
    if (file < 0)
    {
        ROS_ERROR("Unable to open I2C device");
        exit(1);
    }

    buf[0] = reg_address;
    buf[1] = val;

    if (write(file, buf, 2) != 2)
    {
        ROS_ERROR("Unable to write to I2C device");
        exit(1);
    }
}

char i2c_read(uint8_t reg_address)
{
    char buf[1];
    if (file < 0)
    {
        ROS_ERROR("Unable to open I2C device");
        exit(1);
    }

    buf[0] = reg_address;

    if (write(file, buf, 1) != 1)
    {
        ROS_ERROR("Unable to write to I2C device");
        exit(1);
    }

    if (read(file, buf, 1) != 1)
    {
        ROS_ERROR("Unable to read from I2C device");
        exit(1);
    }

    return buf[0];
}

uint16_t merge_bytes(uint8_t LSB, uint8_t MSB)
{
    return (uint16_t)(((LSB & 0xFF) << 8) | MSB);
}

int16_t two_complement_to_int(uint8_t LSB, uint8_t MSB)
{
    int16_t signed_int = 0;
    uint16_t word;

    word = merge_bytes(LSB, MSB);

    if ((word & 0x8000) == 0x8000)
    {
        signed_int = (int16_t) - (~word);
    }
    else
    {
        signed_int = (int16_t)(word & 0x7fff);
    }

    return signed_int;
}

// High-pass filter coefficients
const float alpha = 0.98;
float filtered_x_accel = 0.0;
float filtered_y_accel = 0.0;

void highPassFilter(float &input, float &output)
{
    static float prev_input = input;
    // Apply high-pass filter
    output = alpha * (output + input - prev_input);
    prev_input = input;
}

// Function to calculate the standard deviation of a vector
double calculateStandardDeviation(const std::vector<double> &data)
{
    double mean = 0.0;
    for (const double &value : data)
    {
        mean += value;
    }
    mean /= data.size();

    double variance = 0.0;
    for (const double &value : data)
    {
        variance += std::pow(value - mean, 2);
    }
    variance /= data.size();

    return std::sqrt(variance);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gyro");
    ros::NodeHandle nh;

    char gyro_z_h, gyro_z_l;
    int16_t z_gyro;
    float z_gyro_deg_per_sec;

    char accel_x_h, accel_x_l, accel_y_h, accel_y_l;
    int16_t x_accel, y_accel;
    float x_accel_g, y_accel_g;

    char bus_filename[250];
    uint16_t fifo_len = 0;

    snprintf(bus_filename, 250, "/dev/i2c-0"); // Sesuaikan ini dengan I2C bus yang digunakan
    file = open(bus_filename, O_RDWR);
    if (file < 0)
    {
        ROS_ERROR("Unable to open I2C device");
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, MPU6050_I2C_ADDR) < 0)
    {
        ROS_ERROR("Unable to set I2C slave address");
        return 1;
    }

    i2c_write(REG_PWR_MGMT_1, 0x00);
    i2c_write(REG_PWR_MGMT_1, 0x01);
    i2c_write(REG_SMPRT_DIV, 0x00);
    i2c_write(REG_CONFIG, 0x00);
    i2c_write(REG_ACCEL_CONFIG, 0x10);
    i2c_write(REG_GYRO_CONFIG, 0x10);
    // i2c_write(REG_FIFO_EN, 0x88);
    // i2c_write(REG_USER_CTRL, 0x44);

    int8_t gyro_config = 0;

    gyro_config = i2c_read(REG_GYRO_CONFIG);

    printf("Gyro Config: %d\n", gyro_config);

    ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("yaw_position", 1);
    ros::Publisher yaw_vel_pub = nh.advertise<std_msgs::Float64>("yaw_velocity", 1);
    ros::Publisher mpu_accel = nh.advertise<geometry_msgs::Twist>("mpu_accel", 1);
    ros::Publisher mpu_vel = nh.advertise<geometry_msgs::Twist>("mpu_vel", 1);

    for (uint16_t i = 0; i < 1000; i++)
    {
        accel_x_h = i2c_read(REG_ACCEL_XOUT_H);
        accel_x_l = i2c_read(REG_ACCEL_XOUT_L);
        x_accel = two_complement_to_int(accel_x_h, accel_x_l);
        x_accel_g = ((float)x_accel) / 4096.0 * 9.81;
        // highPassFilter(x_accel_g, filtered_x_accel);

        accel_array[i] = x_accel_g;

        accel_offset_buffer += x_accel_g;

        usleep(1); // Adjust the sleep time as needed
    }

    accel_offset = accel_offset_buffer / 1000.0f;

    printf("Accel offset: %lf\n", accel_offset);

    float buff = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
        buff += (accel_array[i] - accel_offset) * (accel_array[i] - accel_offset);
    }

    accel_standart_deviation = sqrt(buff / 1000.0f);
    // accel_standart_deviation += 0.2;

    printf("Standart deviation: %lf\n", accel_standart_deviation);

    for (uint16_t i = 0; i < 1000; i++)
    {
        gyro_z_h = i2c_read(REG_GYRO_ZOUT_H);
        gyro_z_l = i2c_read(REG_GYRO_ZOUT_L);

        z_gyro = two_complement_to_int(gyro_z_h, gyro_z_l);
        z_gyro_deg_per_sec = ((float)z_gyro) / 32.8;

        gyro_offset_buffer += z_gyro_deg_per_sec;

        usleep(1000);
    }

    gyro_offset = gyro_offset_buffer / 1000.0f;

    printf("Gyro offset: %f\n", gyro_offset);

    while (ros::ok())
    {
        static float gyro_vel[2] = {0, 0};

        gyro_z_h = i2c_read(REG_GYRO_ZOUT_H);
        gyro_z_l = i2c_read(REG_GYRO_ZOUT_L);
        accel_x_h = i2c_read(REG_ACCEL_XOUT_H);
        accel_x_l = i2c_read(REG_ACCEL_XOUT_L);
        accel_y_h = i2c_read(REG_ACCEL_YOUT_H);
        accel_y_l = i2c_read(REG_ACCEL_YOUT_L);

        static double last_time_update = ros::Time::now().toSec();
        double time_now = ros::Time::now().toSec();
        double dt_ = time_now - last_time_update;
        last_time_update = time_now;

        z_gyro = two_complement_to_int(gyro_z_h, gyro_z_l);
        z_gyro_deg_per_sec = ((float)z_gyro) / 32.8;

        x_accel = two_complement_to_int(accel_x_h, accel_x_l);
        y_accel = two_complement_to_int(accel_y_h, accel_y_l);

        // Convert raw values to accelerations in g (assuming the proper sensitivity setting)
        x_accel_g = ((float)x_accel) / 4096.0 * 9.81;
        y_accel_g = ((float)y_accel) / 4096.0 * 9.81;

        x_accel_g -= accel_offset;

        // Apply high-pass filter
        // highPassFilter(x_accel_g, filtered_x_accel);
        // highPassFilter(y_accel_g, filtered_y_accel);

        // filtered_x_accel -= accel_offset;

        filtered_x_accel = alpha * (accel_offset + x_accel_g - accel_offset);

        static float prev_filt_accel = x_accel_g;
        float delta_accel = x_accel_g - prev_filt_accel;
        prev_filt_accel = x_accel_g;

        float filter_filtered_accel_x = 0;

        if (fabs(delta_accel) > accel_standart_deviation)
        {
            filter_filtered_accel_x = alpha * (accel_offset + x_accel_g - accel_offset);
            gyro_vel[0] = 0;
        }

        // filtered_x_accel += -0.04290;
        // filtered_y_accel += -0.0181;

        gyro_vel[0] += filter_filtered_accel_x * dt_;
        gyro_vel[1] += filtered_x_accel * dt_;

        // z_gyro_deg_per_sec += 2.8123f; // Adjust the offset as needed
        // z_gyro_deg_per_sec += 5.2823f; // Adjust the offset as needed
        // z_gyro_deg_per_sec += 2.5823f; // Adjust the offset as needed
        z_gyro_deg_per_sec -= gyro_offset; // Adjust the offset as needed

        static double yaw_pos = 0.0f;

        yaw_pos += z_gyro_deg_per_sec * dt_; // Adjust the sleep time as needed

        while (yaw_pos > 180.0f)
            yaw_pos -= 360.0f;
        while (yaw_pos < -180.0f)
            yaw_pos += 360.0f;

        std_msgs::Float64 msg;
        msg.data = yaw_pos;
        yaw_pub.publish(msg);

        std_msgs::Float64 msg2;
        msg2.data = z_gyro_deg_per_sec;
        yaw_vel_pub.publish(msg2);

        geometry_msgs::Twist msg_accel;
        msg_accel.linear.x = filter_filtered_accel_x;
        msg_accel.linear.y = filtered_x_accel;
        mpu_accel.publish(msg_accel);

        geometry_msgs::Twist msg_vel;
        msg_vel.linear.x = gyro_vel[1];
        msg_vel.linear.y = gyro_vel[0];
        mpu_vel.publish(msg_vel);

        ros::spinOnce();
        usleep(10000); // Adjust the sleep time as needed
    }

    close(file);

    return 0;
}
