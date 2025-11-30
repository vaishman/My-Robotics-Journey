from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld= LaunchDescription()

    control_node_params_file_path= os.path.join(get_package_share_directory("my_robot_bringup"), "config", "ros2_controllers.yaml")
    control_node_robot_description = os.path.join(get_package_share_directory())