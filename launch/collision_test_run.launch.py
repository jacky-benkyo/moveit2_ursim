from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Bypass Builder，execute xacro create URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])
    
    # 2. Bypass Builder，execute xacro create SRDF
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur ur_type:=ur5e"
    ])

    # 3. Activate the Nod with following parameters 
    ur_collision_check_node = Node(
        package="my_moveit_app",
        executable="ur_collision_check_node",
        name="ur_collision_check_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([ur_collision_check_node])