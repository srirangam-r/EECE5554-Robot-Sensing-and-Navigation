from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB0',
        description='The port where the IMU is connected'
    )

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB1',
        description='The port where the GPS device is connected'
    )

    imu_driver_node = Node(
        package='imu_driver',  
        executable='driver',  
        name='imu_driver',
        parameters=[{
            'port': LaunchConfiguration('imu_port') 
        }]
    )

    gps_driver_node = Node(
        package='gps_driver',  
        executable='driver',  
        name='gps_driver',
        parameters=[{
            'port': LaunchConfiguration('gps_port')  
        }]
    )

    return LaunchDescription([
        imu_port_arg,
        gps_port_arg,
        imu_driver_node,
        gps_driver_node
    ])
