from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='200.0',
            description='IMU publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for IMU messages'
        ),
        DeclareLaunchArgument(
            'i2c_device',
            default_value='/dev/i2c-7',
            description='I2C device path (your device)'
        ),
        DeclareLaunchArgument(
            'calibrate_on_startup',
            default_value='true',
            description='Calibrate gyroscope on startup'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error)'
        ),
        
        Node(
            package='icm20948_ros2',
            executable='icm20948_ros2_node',
            name='icm20948_node',
            parameters=[{
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'frame_id': LaunchConfiguration('frame_id'),
                'i2c_device': LaunchConfiguration('i2c_device'),
                'calibrate_on_startup': LaunchConfiguration('calibrate_on_startup'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        )
    ])
