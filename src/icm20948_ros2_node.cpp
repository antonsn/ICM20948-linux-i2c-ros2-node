#include "icm20948_ros2_node.hpp"

ICM20948ROS2Node::ICM20948ROS2Node() : Node("icm20948_node")
{
    // Declare parameters with defaults from your Config
    this->declare_parameter("publish_rate_hz", 200.0);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("calibrate_on_startup", true);
    this->declare_parameter("i2c_device", "/dev/i2c-7");  // Your actual device
    
    // Get parameters
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    calibrate_on_startup_ = this->get_parameter("calibrate_on_startup").as_bool();
    
    // Initialize covariance matrices for OpenVINS (tune based on your calibration)
    angular_velocity_covariance_ = {
        0.001, 0.0,   0.0,
        0.0,   0.001, 0.0,
        0.0,   0.0,   0.001
    };
    
    linear_acceleration_covariance_ = {
        0.01, 0.0,  0.0,
        0.0,  0.01, 0.0,
        0.0,  0.0,  0.01
    };
    
    orientation_covariance_ = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
    };
    
    // Create publisher for OpenVINS raw IMU topic
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/icm20948/raw", 10);
    
    // Initialize ICM20948 sensor
    imu_sensor_ = std::make_unique<ICM20948>();
    
    // Configure sensor for OpenVINS using your existing Config structure
    ICM20948::Config config = createOpenVINSConfig();
    
    // Set I2C device from parameter
    std::string i2c_device = this->get_parameter("i2c_device").as_string();
    strncpy(config.mDevice, i2c_device.c_str(), sizeof(config.mDevice) - 1);
    config.mDevice[sizeof(config.mDevice) - 1] = '\0';
    
    if (imu_sensor_->initialise(config))
    {
        RCLCPP_INFO(this->get_logger(), "ICM20948 initialized successfully on %s", config.mDevice);
        
        if (calibrate_on_startup_)
        {
            RCLCPP_INFO(this->get_logger(), "Performing gyroscope calibration...");
            imu_sensor_->calibrateGyro();
            RCLCPP_INFO(this->get_logger(), "Gyroscope calibration completed");
        }
        
        // Create timer for publishing at specified rate
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&ICM20948ROS2Node::timerCallback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Publishing IMU data at %.1f Hz to topic /icm20948/raw", publish_rate_hz_);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize ICM20948 sensor");
        rclcpp::shutdown();
    }
}

ICM20948ROS2Node::~ICM20948ROS2Node()
{
    RCLCPP_INFO(this->get_logger(), "ICM20948 ROS2 node shutting down");
}

ICM20948::Config ICM20948ROS2Node::createOpenVINSConfig()
{
    ICM20948::Config config;
    
    // Configure for high-rate operation suitable for OpenVINS
    config.mFramerate = static_cast<float>(publish_rate_hz_);
    
    // Gyroscope configuration for OpenVINS
    config.mGyro.mEnabled = true;
    config.mGyro.mRange = ICM20948::GYRO_RANGE_250DPS; // Good range for VIO
    config.mGyro.mSampleRateDivisor = 0; // Maximum rate
    config.mGyro.mDLPFBandwidth = ICM20948::GYRO_DLPF_BANDWIDTH_120HZ; // Good bandwidth for VIO
    config.mGyro.mAveraging = ICM20948::GYRO_AVERAGING_1X; // No averaging for max rate
    
    // Accelerometer configuration for OpenVINS
    config.mAcc.mEnabled = true;
    config.mAcc.mRange = ICM20948::ACC_RANGE_4G; // Good range for VIO
    config.mAcc.mSampleRateDivisor = 0; // Maximum rate
    config.mAcc.mDLPFBandwidth = ICM20948::ACC_DLPF_BANDWIDTH_111HZ; // Good bandwidth for VIO
    config.mAcc.mAveraging = ICM20948::ACC_AVERAGING_NONE; // No averaging for max speed
    
    // Temperature sensor
    config.mTemp.mEnabled = true;
    
    // Disable magnetometer for higher rates (OpenVINS doesn't need it for VIO)
    config.mMagEnabled = false;
    
    // Use Madgwick filter for orientation estimation
    config.mAHRS = ICM20948::NONE;
    
    return config;
}

void ICM20948ROS2Node::timerCallback()
{
    // Get latest IMU data using your existing method
    const IMUData& data = imu_sensor_->imuDataGet();
    
    // Publish IMU message
    publishIMUData(data);
}

void ICM20948ROS2Node::publishIMUData(const IMUData& data)
{
    auto imu_msg = sensor_msgs::msg::Imu();
    
    // Set header
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;
    
    // Set orientation (quaternion from AHRS)
    imu_msg.orientation.w = data.mQuat[0];
    imu_msg.orientation.x = data.mQuat[1];
    imu_msg.orientation.y = data.mQuat[2];
    imu_msg.orientation.z = data.mQuat[3];
    
    // Set angular velocity (rad/s) - already in rad/s from your sensor
    imu_msg.angular_velocity.x = data.mGyro[0];
    imu_msg.angular_velocity.y = data.mGyro[1];
    imu_msg.angular_velocity.z = data.mGyro[2];
    
    // Set linear acceleration (m/s²) - convert from G to m/s²
    const double g = 9.80665; // Standard gravity
    imu_msg.linear_acceleration.x = data.mAcc[0] * g;
    imu_msg.linear_acceleration.y = data.mAcc[1] * g;
    imu_msg.linear_acceleration.z = data.mAcc[2] * g;
    
    // Set covariance matrices
    for (size_t i = 0; i < 9; ++i) {
        imu_msg.orientation_covariance[i] = orientation_covariance_[i];
        imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance_[i];
        imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance_[i];
    }
    
    // Debug logging - show IMU values when debug level is enabled
    RCLCPP_DEBUG(this->get_logger(), 
        "IMU Data - Accel(m/s²): [%.3f, %.3f, %.3f], Gyro(rad/s): [%.3f, %.3f, %.3f], "
        "Quat: [%.3f, %.3f, %.3f, %.3f], Angles(°): [%.1f, %.1f, %.1f], Temp: %.1f°C",
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
        imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z,
        data.mAngles[0], data.mAngles[1], data.mAngles[2], data.mTemp);
    
    // Publish the message
    imu_publisher_->publish(imu_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ICM20948ROS2Node>();
    
    RCLCPP_INFO(node->get_logger(), "Starting ICM20948 ROS2 node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
