from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='The port where the GPS device is connected'
    )

    # Configure the GPS driver node
    gps_driver_node = Node(
        package='gps_driver',  # Name of your GPS driver package
        executable='driver',  # Name of the executable
        name='gps_driver',
        parameters=[{
            'port': LaunchConfiguration('port')  # Pass the port argument to the node
        }]
    )

    return LaunchDescription([
        port_arg,
        gps_driver_node
    ])
