from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

  
    return LaunchDescription([
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_esp32_Node",
            arguments=['serial', '--dev', '/dev/esp32'],
            output='screen',
        ),
        Node(
            package='omega',
            executable='core',
            name='Core_Node',
            output='screen',
        ),
        Node(
            package='omega',
            executable='rfid',
            name='RFID_Reader_Node',
            output='screen',
        ),
        
        # Node(
        #     package='omega',
        #     executable='data_pub',
        #     name='DATA_Sender',
        #     output='screen',
        # ),
        # Node(
        #     package='omega',
        #     executable='data_sub',
        #     name='DATA_Reader',
        #     output='screen',
        # ),

        Node(
            package='joy',
            executable='joy_node',
            name='PS4_Controller_Node',
            output='both',
        ),

    ])
