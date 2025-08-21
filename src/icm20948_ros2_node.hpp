#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <memory>
#include "icm20948.h"

class ICM20948ROS2Node : public rclcpp::Node
{
public:
    ICM20948ROS2Node();
    ~ICM20948ROS2Node();

private:
    void timerCallback();
    void publishIMUData(const IMUData& data);
    ICM20948::Config createOpenVINSConfig();

    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // ICM20948 sensor
    std::unique_ptr<ICM20948> imu_sensor_;
    
    // Configuration
    double publish_rate_hz_;
    std::string frame_id_;
    bool calibrate_on_startup_;
    
    // Covariance matrices for OpenVINS
    std::array<double, 9> angular_velocity_covariance_;
    std::array<double, 9> linear_acceleration_covariance_;
    std::array<double, 9> orientation_covariance_;
};
