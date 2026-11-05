import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'balance_world'


def generate_launch_description():
    pkg_omni_pendulum = get_package_share_directory('omni_pendulum_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_omni_pendulum, 'urdf', 'robot.urdf.xacro')
    world = os.path.join(pkg_omni_pendulum, 'worlds', f'{WORLD_NAME}.sdf')
    gui_config = os.path.join(pkg_omni_pendulum, 'config', 'gui.config')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gui = LaunchConfiguration('gui')
    push = LaunchConfiguration('push')
    balance = LaunchConfiguration('balance')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Start the Gazebo GUI')
    declare_push = DeclareLaunchArgument(
        'push', default_value='true', description='Start-up nudge and manual nudges')
    declare_balance = DeclareLaunchArgument(
        'balance', default_value='true',
        description='Run the balance controller (false = watch the pole fall)')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo server and GUI in one process, so the GUI always shows this world;
    # gui:=false runs the server only (-s)
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r -v 3 {world} --gui-config {gui_config}',
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui)
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r -s -v 3 {world}', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(gui)
    )

    # base_footprint is at ground level; spawn just above it so the landing
    # does not knock the pendulum over
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', WORLD_NAME, '-topic', 'robot_description',
                   '-name', 'omni_pendulum', '-z', '0.005']
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/keyboard/keypress@std_msgs/msg/Int32[gz.msgs.Int32',
            f'/world/{WORLD_NAME}/wrench/persistent'
            '@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            f'/world/{WORLD_NAME}/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    node_balance_controller = Node(
        package='omni_pendulum_sim',
        executable='balance_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(balance)
    )

    node_disturbance = Node(
        package='omni_pendulum_sim',
        executable='disturbance',
        parameters=[{'use_sim_time': True, 'world': WORLD_NAME}],
        output='screen',
        condition=IfCondition(push)
    )

    return LaunchDescription([
        declare_gui,
        declare_push,
        declare_balance,
        node_robot_state_publisher,
        gz_sim,
        gz_server,
        node_ros_gz_bridge,
        node_balance_controller,
        node_disturbance,
        # Give Gazebo and robot_state_publisher a moment before spawning
        TimerAction(period=2.0, actions=[node_spawn_entity]),
    ])
