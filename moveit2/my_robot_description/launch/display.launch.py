from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld=LaunchDescription()
    
    urdf_path=os.path.join(get_package_share_directory("my_robot_description"),"urdf","my_robot.urdf.xacro")
    rviz_config_path=os.path.join(get_package_share_directory("my_robot_description"),"rviz","urdf_config.rviz")
    
    robot_description=Command(['xacro ', urdf_path])
    
    robot_state_publisher=Node(
        package="robot_state_publisher",
        name="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        # name="joint_state_publisher_GUI",
        executable="joint_state_publisher_gui",

    )

    rviz=Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path]
        )
    
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    return ld


