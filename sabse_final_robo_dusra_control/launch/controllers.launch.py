import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('sabse_final_robo_dusra_description')
    pkg_control = get_package_share_directory('sabse_final_robo_dusra_control')
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_description, 'urdf', 'sabse_final_robo_dusra.xacro')
    robot_description_raw = xacro.process_file(xacro_file, mappings={'use_ros2_control': 'true'})
    robot_description = {'robot_description': robot_description_raw.toxml()}
    
    # Load robot controllers
    robot_controllers = os.path.join(
        pkg_control,
        'config',
        'sabse_final_robo_dusra_controllers.yaml'
    )
    
    # Define nodes
    nodes = []
    
    # 1. Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes.append(robot_state_pub_node)
    
    # 2. Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    nodes.append(control_node)
    
    # 3. Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    nodes.append(joint_state_broadcaster_spawner)

    # 4. Arm Controller Spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # 5. Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # 6. Delay arm controller start after joint state broadcaster
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    nodes.append(delay_arm_controller_spawner)
    
    # 7. Delay gripper controller start after arm controller
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    nodes.append(delay_gripper_controller_spawner)

    # 8. RViz
    rviz_config_file = os.path.join(pkg_description, 'config', 'robot_config.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    nodes.append(rviz_node)

    return LaunchDescription(nodes)