from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Generate URDF via xacro 
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])
    
    # 2. Generate SRDF via xacro
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])

    # 3. Activate MNode with parameters
    ur_test_node = Node(
        package="my_moveit_app",
        executable="ur_manipulator_node",
        name="ur_test_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([ur_test_node])