import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. Bypass Builder, execute xacro to create URDF (Robot Kinematics/Structure)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])
    
    # 2. Bypass Builder, execute xacro to create SRDF (Semantic/Joint Groups Configuration)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])

    # 3. Set absolute path && load parameters from yaml
    pkg_share = get_package_share_directory('my_moveit_app')
    params_yaml_path = os.path.join(pkg_share, 'config', 'motion_params.yaml')

    # 4. Activate the Node with following parameters 
    #Launch - Accept both Key-Value Dictionary {"A": A_content} and Plain String Path
    ur_collision_check_node = Node(
        package="my_moveit_app",
        executable="ur_collision_check_node",
        name="ur_collision_check_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"use_sim_time": False},
            params_yaml_path
        ],
    )
    #Manual Command 
    #Load yaml file 
    #ros2 run my_moveit_app ur_collision_check_node --ros-args --params-file src/my_moveit_app/config/motion_params.yaml

    #Adjust individual parameter value (online)
    #ros2 param set /ur_collision_check retraction_height 0.15

    return LaunchDescription([ur_collision_check_node])