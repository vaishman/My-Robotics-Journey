"""from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Paths ---
    urdf_path = os.path.join(
        get_package_share_directory("my_robot_description"),
        "urdf",
        "my_robot.urdf.xacro"
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config",
        "my_robot_moveit.rviz"
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("my_robot_bringup"),
        "config",
        "ros2_controllers.yaml"
    )

    moveit_launch_file = os.path.join(
        get_package_share_directory("my_robot_moveit_config"),
        "launch",
        "move_group.launch.py"
    )

    # --- Nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="ros2_control_node",
        parameters=[controllers_yaml],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output='screen'
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file)
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # --- Launch Description ---
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_launch,
        rviz
    ])
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart



def generate_launch_description():
    ld= LaunchDescription()
   # robot_description:="$(xacro /home/leo/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)" 
    robot_description_path = os.path.join(get_package_share_directory("my_robot_description"),"urdf","my_robot.urdf.xacro")
    robot_description = Command(['xacro ', robot_description_path])

    controller_manager_params_file_path = os.path.join(get_package_share_directory("my_robot_bringup"),"config","ros2_controllers.yaml")

    rviz_config_path = os.path.join(get_package_share_directory("my_robot_bringup"),"rviz","urdf_config.rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        name="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    control_node = Node(
        package="controller_manager",
        name="ros2_control_node",
        executable="ros2_control_node",
        parameters=[controller_manager_params_file_path, {'robot_description': robot_description}] 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )


    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )


    Gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Gripper_controller"],
        output="screen"
    )


    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("my_robot_moveit_config"),"launch","move_group.launch.py")
        ])
    )

    rviz = Node(
        package="rviz2",
        name="rviz",
        executable= "rviz2",
        output = "screen",
        arguments=['-d', rviz_config_path]
    )

  #  ld.add_action(robot_state_publisher)
    ld.add_action(control_node)
 # Launch order
    ld.add_action(robot_state_publisher)
    ld.add_action(control_node)

    # Spawn controllers after ros2_control_node starts
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner, arm_spawner, Gripper_spawner]
        )
    ))
    # ld.add_action(RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=control_node,
    #         on_exit=[joint_state_broadcaster_spawner]
    #     )
    # ))

    # ld.add_action(RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[arm_spawner, Gripper_spawner]
    #     )
    # ))


    # ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(arm_spawner)
    # ld.add_action(Gripper_spawner)


    ld.add_action(move_group_launch)
    ld.add_action(rviz)
    
    return ld




"""ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/leo/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"


 ros2 run controller_manager ros2_control_node --ros-args \
  --params-file ~/ros2_ws/src/my_robot_bringup/config/ros2_controllers.yaml \
  -p robot_description:="$(xacro ~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"





ros2 run controller_manager spawner joint_state_broadcaster


ros2 run controller_manager spawner arm_controller

leo@leo:~$ ros2 run controller_manager spawner Gripper_controller


leo@leo:~$ ros2 launch my_robot_moveit_config move_group.launch.py 
leo@leo:~$ ros2 run rviz2 rviz2 -d ~/ros2_ws/src/my_robot_description/rviz/urdf_config.rviz 
"""