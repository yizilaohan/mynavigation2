"""Navigation entry: external localization exclusively publishes map -> base_link.

Subscribe to /scan with the base_link -> base_scan extrinsics. Raw /odometry
is converted to /motion_state for controllers, the navigator and the smoother.
All modes default to headless Gazebo with its feedback bridges.
use_gazebo:=false switches to external sensor and velocity inputs.
No Gazebo bottom-chassis TF or substitute localization is published here.

use_collision_monitor:=true starts the collision detection module: dynamic
zones (car_op_zones.py --drive-monitor), the default status source, and (when
use_obstacle_status is also true) the status enumerator. The module does not
change velocity. use_obstacle_status defaults true and makes the translator
publish /collision_state (UInt8) and /obstacle_status_int (Int32); it runs with
--no-passthrough and never publishes /cmd_vel. The Nav2 collision_monitor
velocity gate is separate and stays off unless use_collision_velocity_gate is
true.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_ign_partition = 'myworld_bringup_{}_{}'.format(os.environ.get('ROS_DOMAIN_ID', '0'), os.getpid())
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ign_partition_name = LaunchConfiguration('ign_partition', default=default_ign_partition)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    operation_mode = LaunchConfiguration('operation_mode', default='autonomous')
    # Keep the simulator server and sensor bridge active without opening a Gazebo GUI.
    default_use_gazebo = 'true'
    default_collision_visualization = PythonExpression(["'true' if '", operation_mode, "' == 'fixed_path' else 'false'"])
    use_gazebo = LaunchConfiguration('use_gazebo', default=default_use_gazebo)
    default_collision_monitor = 'false'
    # use_collision_monitor 是碰撞检测模块（状态枚举链）总开关；速度门单独用
    # use_collision_velocity_gate，二者互不影响。
    use_collision_monitor = LaunchConfiguration('use_collision_monitor', default=default_collision_monitor)
    use_collision_velocity_gate = LaunchConfiguration('use_collision_velocity_gate', default='false')
    use_obstacle_status = LaunchConfiguration('use_obstacle_status', default='true')
    use_car_op_status_sim = LaunchConfiguration('use_car_op_status_sim', default='true')
    # ooii: 新接口 RobotMoveState（uint8 九值 0–8）；默认 7 = 静止。
    car_op_status_value = LaunchConfiguration('car_op_status_value', default='7')
    car_op_status_topic = LaunchConfiguration('car_op_status_topic', default='/robot_move_state')
    use_collision_visualization = LaunchConfiguration('use_collision_visualization', default=default_collision_visualization)
    headless = LaunchConfiguration('headless', default='true')
    navigation_stack_start_delay = LaunchConfiguration('navigation_stack_start_delay', default='3.0')

    is_navigation = PythonExpression(["'", operation_mode, "' != 'remote'"])
    gazebo_enabled = ["'", use_gazebo, "'.lower() == 'true' and "]
    is_autonomous_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' in ('autonomous', 'autonomous_fixed') and '", headless, "' == 'false'"])
    is_autonomous_headless = PythonExpression(gazebo_enabled + ["'", operation_mode, "' in ('autonomous', 'autonomous_fixed') and '", headless, "' == 'true'"])
    is_fixed_path_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'fixed_path' and '", headless, "' == 'false'"])
    is_remote_gui = PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'remote' and '", headless, "' == 'false'"])

    bringup_dir = get_package_share_directory('myworld_bringup')
    myworld_dir = os.path.join(bringup_dir, 'models', 'myworld2')
    default_map = PythonExpression([
        "'", os.path.join('/userdata/map/pbstream', 'out.yaml'),
        "' if '", operation_mode, "' in ('fixed_path', 'autonomous', 'autonomous_fixed') else '",
        os.path.join(myworld_dir, 'myworld2.yaml'), "'"])
    map_file = LaunchConfiguration('map', default=default_map)
    rviz_config_file = PythonExpression([
        "'", os.path.join(bringup_dir, 'rviz', 'nav2_autonomous_fixed_view.rviz'),
        "' if '", operation_mode, "' == 'autonomous_fixed' else ",
        "'", os.path.join(bringup_dir, 'rviz', 'nav2_autonomous_view.rviz'),
        "' if '", operation_mode, "' == 'autonomous' else '",
        os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'), "'"])
    robot_description_file = os.path.join(bringup_dir, 'urdf', 'diffbot.urdf')
    with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
        robot = ET.fromstring(urdf_file.read())
    # External localization owns map -> base_link. Keep base_link a URDF root;
    # publishing base_footprint -> base_link would give base_link two parents.
    base_joint = robot.find("joint[@name='base_joint']")
    origin = base_joint.find('origin')
    if origin.get('xyz') != '0 0 0' or origin.get('rpy') != '0 0 0':
        raise ValueError('External localization expects the existing zero-offset base_joint')
    base_joint.find('parent').set('link', 'base_link')
    base_joint.find('child').set('link', 'base_footprint')
    # Fixed sensor joints publish on /tf_static; wheel joints follow /joint_states on /tf.
    robot_description = ET.tostring(robot, encoding='unicode')

    resource_path = [os.path.join('/opt/ros/humble', 'share'), ':' + os.path.join(bringup_dir, 'models'), ':' + myworld_dir, ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path)
    ign_partition = SetEnvironmentVariable(name='IGN_PARTITION', value=ign_partition_name)

    world_only = LaunchConfiguration('world_file', default=os.path.join(myworld_dir, 'world_only.sdf'))
    ignition_sim = ExecuteProcess(condition=IfCondition(is_autonomous_gui), cmd=['ign', 'gazebo', '-r', '-v', '3', world_only], output='screen')
    ignition_server = ExecuteProcess(condition=IfCondition(is_autonomous_headless), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    fixed_path_ignition_sim = ExecuteProcess(condition=IfCondition(is_fixed_path_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    fixed_path_ignition_server = ExecuteProcess(condition=IfCondition(PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'fixed_path'"])), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')
    remote_ignition_gui = ExecuteProcess(condition=IfCondition(is_remote_gui), cmd=['ign', 'gazebo', '-g', '-v', '3', world_only], output='screen')
    remote_ignition_server = ExecuteProcess(condition=IfCondition(PythonExpression(gazebo_enabled + ["'", operation_mode, "' == 'remote'"])), cmd=['ign', 'gazebo', '-r', '-s', '--headless-rendering', '-v', '3', world_only], output='screen')

    algorithm_arguments = {'map': map_file,
        'use_sim_time': use_sim_time,
        'params_file': LaunchConfiguration('params_file'),
        'rviz_config_file': rviz_config_file,
        'use_rviz': use_rviz,
        'namespace': '',
        'use_namespace': 'false',
        # regulated 的 use_collision_monitor 现在也是状态模块总开关；状态模块已在
        # 下面的 Node 里独立启动，这里必须关掉避免重复节点。速度门单独转发。
        'use_collision_monitor': 'false',
        'use_collision_velocity_gate': use_collision_velocity_gate,
        'use_collision_visualization': use_collision_visualization,
        'use_obstacle_status': 'false',
        'autostart': 'true',
        'use_composition': 'False',
        'container_name': 'nav2_regulated_container',
        'use_respawn': 'False',
        'log_level': 'info',
        'operation_mode': operation_mode,
        # Scene planner selection stays in params_file, rather than the child Launch default.
        'autonomous_planner_id': '',
        'autonomous_mppi_profile': LaunchConfiguration('autonomous_mppi_profile'),
        'use_path_smoother': PythonExpression(["'", operation_mode, "' not in ('fixed_path', 'autonomous_fixed')"]),
    }

    # 两种导航模式共用地图、外部定位、反馈接口和启动顺序，由 operation_mode 选择任务流程。
    # 只有一个 RViz，由 regulated 的 rviz_launch.py 负责；use_rviz 透传给它。
    # remote 模式由 navigation_group 的 is_navigation 条件兜底，不会启动。
    regulated_navigation = TimerAction(
        period=navigation_stack_start_delay, condition=IfCondition(is_navigation),
        actions=[GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_regulated_modules'), '/launch/regulated_modules.launch.py']),
            launch_arguments=algorithm_arguments.items())])])

    return LaunchDescription([
        ign_resource_path,
        ign_partition,
        ignition_sim,
        ignition_server,
        fixed_path_ignition_sim,
        fixed_path_ignition_server,
        remote_ignition_gui,
        remote_ignition_server,

        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'use_sim_time': use_sim_time,
                'robot_description': robot_description}]),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use system timestamps by default; true restores Gazebo simulation time'),
        DeclareLaunchArgument('map', default_value=default_map, description='Navigation modes share /userdata/map/pbstream/out.yaml; remote uses myworld2; explicit map overrides the default'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'myworld2.yaml'), description='Explicit simulation parameter file for isolated experiments'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(myworld_dir, 'world_only.sdf'), description='Explicit simulation SDF for isolated experiments'),
        DeclareLaunchArgument('ign_partition', default_value=default_ign_partition, description='Unique Ignition transport partition for this launch'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('use_gazebo', default_value=default_use_gazebo, description='Start Gazebo, sensor bridges and simulated velocity feedback; defaults true in all modes, with the GUI disabled by headless=true'),
        DeclareLaunchArgument('use_collision_monitor', default_value=default_collision_monitor, description='Start the collision detection module (dynamic zones + default status source); it publishes status only and never changes velocity'),
        DeclareLaunchArgument('use_collision_velocity_gate', default_value='false', description='Start the Nav2 collision_monitor velocity gate separately; keep false when the status module is the intended collision behaviour'),
        DeclareLaunchArgument('use_collision_visualization', default_value=default_collision_visualization, description='Enable collision boundary visualization; defaults true for fixed_path, false for other modes'),
        DeclareLaunchArgument('use_obstacle_status', default_value='true', description='With use_collision_monitor: start obstacle_status_translator and publish /collision_state (UInt8) and /obstacle_status_int (Int32). The translator never publishes /cmd_vel'),
        DeclareLaunchArgument('use_car_op_status_sim', default_value='true', description='With use_collision_monitor: start the default RobotMoveState source. It yields while another publisher on car_op_status_topic exists and resumes afterwards'),
        DeclareLaunchArgument('car_op_status_value', default_value='7', description='ooii: car_op_status_sim hold robot_move_state 0-8 (7 = stationary)'),
        DeclareLaunchArgument('car_op_status_topic', default_value='/robot_move_state', description='ooii: byd_custom_msgs/RobotMoveState topic shared by the default source and car_op_zones'),
        DeclareLaunchArgument('headless', default_value='true', description='Run only the Ignition Gazebo server by default; false enables Gazebo GUI'),
        DeclareLaunchArgument('operation_mode', default_value='autonomous', choices=['remote', 'autonomous', 'fixed_path', 'autonomous_fixed'], description='Select the robot operation mode'),
        DeclareLaunchArgument('autonomous_mppi_profile', default_value='true', description='Apply rotation alignment, terminal approach and reduced MPPI sampling only in autonomous mode'),
        DeclareLaunchArgument('navigation_stack_start_delay', default_value=navigation_stack_start_delay, description='Shared stack delay for sensors and external localization (s)'),

        # Only sensors and velocity feedback cross the bridge. No Gazebo TF is
        # published; external localization must provide map -> base_link.
        # AMCL 加载保持注释／禁用：不引入 localization_launch.py，由外部定位提供 TF。
        Node(condition=IfCondition(use_gazebo), package='ros_gz_bridge', executable='parameter_bridge', name='simulation_sensor_bridge',
             output='screen', parameters=[{'use_sim_time': use_sim_time,
                 'override_timestamps_with_wall_time': PythonExpression(["'", use_sim_time, "'.lower() == 'false'"])}],
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
             ], remappings=[('/odom', '/odometry')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_clock_bridge',
             condition=IfCondition(PythonExpression(gazebo_enabled + ["'", use_sim_time, "'.lower() == 'true'"])), parameters=[{'use_sim_time': True}],
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),

        Node(condition=IfCondition(use_gazebo), package='myworld_bringup', executable='odom_to_motion_state.py', name='odom_to_motion_state', output='screen', parameters=[{'use_sim_time': use_sim_time,
                'input_topic': '/odometry',
                'output_topic': '/motion_state'}]),
        # 碰撞检测模块：use_collision_monitor 是总开关；模块只发状态，不改速度。
        # use_obstacle_status 默认 true，决定是否启动 translator 输出枚举：
        # /collision_state (UInt8) 给 regulated，/obstacle_status_int (Int32) 给运控
        # （见 nav2_collision_monitor/doc/OBSTACLE_STATUS.md）。
        Node(condition=IfCondition(use_collision_monitor),
             package='nav2_collision_monitor', executable='car_op_zones.py',
             name='car_op_zones', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--drive-monitor', '--frame-id', 'base_link', '--period', '0.1',
                        '--profile', 'sim',
                        # ooii: 新接口 RobotMoveState；话题由 car_op_status_topic 统一。
                        '--op-topic', car_op_status_topic]),
        # 默认状态源保住启动时的状态 0 边界框；它带 --yield-to-other-publishers，
        # 窗口 4 再起 car_op_status_sim 时会自动让位，退出后恢复，
        # 避免两个发布者同时发导致动态框跳闪。
        Node(condition=IfCondition(PythonExpression([
                 "'", use_collision_monitor, "'.lower() == 'true' and '",
                 use_car_op_status_sim, "'.lower() == 'true'"])),
             package='nav2_collision_monitor', executable='car_op_status_sim.py',
             name='car_op_status_sim', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--mode', 'hold', '--value', car_op_status_value,
                        '--topic', car_op_status_topic,
                        '--yield-to-other-publishers']),
        Node(condition=IfCondition(PythonExpression([
                 "'", use_collision_monitor, "'.lower() == 'true' and '",
                 use_obstacle_status, "'.lower() == 'true'"])),
             package='nav2_collision_monitor', executable='obstacle_status_translator.py',
             name='obstacle_status_translator', output='screen',
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['--scan-topic', '/scan',
                        '--stop-topic', '/car_op_zones/stop',
                        '--slow-l2-topic', '/car_op_zones/slow_l2',
                        '--slow-l1-topic', '/car_op_zones/slow_l1',
                        '--zone-frame', 'base_link',
                        '--profile', 'sim',
                        '--no-passthrough']),
        regulated_navigation,

        # 每次启动必定重定位：延迟后向外部定位发布初始位姿种子（模拟 RViz 手动
        # 2D Pose Estimate），短期内重复发送以免外部定位未就绪时错过，随后节点退出。
        # 位姿与发送参数全部配置在 scripts/initial_pose_pub.py，不在此暴露参数。
        TimerAction(period=8.0, actions=[Node(package='myworld_bringup',
                    executable='initial_pose_pub.py', name='initial_pose_pub', output='screen'),
            ]),
    ])
