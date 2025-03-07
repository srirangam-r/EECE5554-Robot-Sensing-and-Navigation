from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='The port where the IMU is connected'
    )

    
    imu_driver_node = Node(
        package='imu_driver',  
        executable='driver',  
        name='imu_driver',
        parameters=[{
            'port': LaunchConfiguration('port')  
        }]
    )

    return LaunchDescription([
        port_arg,
        imu_driver_node
    ])
