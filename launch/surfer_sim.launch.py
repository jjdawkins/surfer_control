
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    agents = ["alpha"]
    agent = 'billy'
    # Bridge
    bridge = Node(
        namespace='my_surfer',
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/'+agent+'/joint/propeller_joint_fr/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/model/'+agent+'/joint/propeller_joint_fl/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/model/'+agent+'/joint/propeller_joint_br/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
                   '/model/'+agent+'/joint/propeller_joint_bl/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double'],
        output='screen'
    )

    return LaunchDescription([
        bridge
    ])
