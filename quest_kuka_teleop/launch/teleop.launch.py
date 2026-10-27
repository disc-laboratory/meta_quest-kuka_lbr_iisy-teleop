"""Servo + Quest bridge (+ ROS-TCP endpoint or fake Quest). Run after robot.launch.py
is ACTIVE. Publishes only to /servo_node/pose_target_cmds; Servo publishes to the JTC."""
import yaml
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    share = get_package_share_directory('quest_kuka_teleop')
    tcp = yaml.safe_load(open(share + '/config/tcp.yaml'))
    xyz, rpy = ' '.join(map(str, tcp['tcp_xyz'])), ' '.join(map(str, tcp['tcp_rpy']))
    collision_model = LaunchConfiguration('workspace_collision_model').perform(context).lower()
    geometry_args = ''
    if collision_model == 'true':
        radius = float(LaunchConfiguration('tool_envelope_radius').perform(context))
        length = float(LaunchConfiguration('tool_envelope_length').perform(context))
        table_z = float(LaunchConfiguration('table_top_z').perform(context))
        if not all(map(math.isfinite, (radius, length, table_z))) or radius <= 0 or length <= 0:
            raise ValueError('Measured positive tool envelope dimensions are required for workspace collision model')
        geometry_args = (f' workspace_collision_model:=true tool_envelope_radius:={radius}'
                         f' tool_envelope_length:={length} table_top_z:={table_z}')
    desc = {'robot_description': ParameterValue(Command(
        ['xacro ', share + '/urdf/lbr_iisy3_r760.urdf.xacro', f' tcp_xyz:="{xyz}" tcp_rpy:="{rpy}"', geometry_args]), value_type=str)}
    srdf = open(share + '/config/lbr_iisy3_r760.srdf').read()
    if collision_model == 'true':
        # Only the mounting link may overlap the padded attached-tool envelope.
        srdf = srdf.replace('</robot>', '<disable_collisions link1="link_6" link2="tool_envelope" reason="Adjacent"/></robot>')
    semantic = {'robot_description_semantic': srdf}
    kin = {'robot_description_kinematics': yaml.safe_load(open(share + '/config/kinematics.yaml'))}
    servo_params = yaml.safe_load(open(LaunchConfiguration('servo_config').perform(context)))['servo_node']['ros__parameters']
    fake = LaunchConfiguration('fake_quest')
    endpoint = LaunchConfiguration('endpoint')
    gripper_backend = LaunchConfiguration('gripper_backend').perform(context)
    if gripper_backend not in ('mock', 'relay'):
        raise ValueError('gripper_backend must be mock or relay')
    gripper_parameters = ({
        'relay_port': LaunchConfiguration('relay_port').perform(context),
        'relay_open_channel': int(LaunchConfiguration('relay_open_channel').perform(context)),
        'relay_close_channel': int(LaunchConfiguration('relay_close_channel').perform(context)),
        'relay_pulse_duration': float(LaunchConfiguration('relay_pulse_duration').perform(context)),
    } if gripper_backend == 'relay' else {'backend': 'mock'})
    return [
        Node(package='tf2_ros', executable='static_transform_publisher', name='gripper_tcp_tf',
             arguments=['--frame-id', 'tool0', '--child-frame-id', 'gripper_tcp',
                        *sum([[f'--{k}', str(v)] for k, v in zip(('x', 'y', 'z'), tcp['tcp_xyz'])], []),
                        *sum([[f'--{k}', str(v)] for k, v in zip(('roll', 'pitch', 'yaw'), tcp['tcp_rpy'])], [])]),
        Node(package='moveit_servo', executable='servo_node', name='servo_node', output='screen',
             parameters=[servo_params, desc, semantic, kin]),
        Node(package='quest_kuka_teleop', executable='bridge', name='quest_bridge', output='screen',
             parameters=[LaunchConfiguration('bridge_config').perform(context),
                         {'translation_only': ParameterValue(LaunchConfiguration('translation_only'), value_type=bool)}]),
        Node(package='quest_kuka_teleop', executable='fake_quest', condition=IfCondition(fake)),
        Node(package='quest_kuka_teleop',
             executable='relay_gripper' if gripper_backend == 'relay' else 'gripper_io',
             name='gripper_io', output='screen', parameters=[gripper_parameters],
             condition=IfCondition(LaunchConfiguration('gripper'))),
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', condition=IfCondition(endpoint),
             parameters=[{'ROS_IP': LaunchConfiguration('endpoint_ip'), 'ROS_TCP_PORT': 10000}]),
    ]


def generate_launch_description():
    share = get_package_share_directory('quest_kuka_teleop')
    return LaunchDescription([
        DeclareLaunchArgument('workspace_collision_model', default_value='false'),
        DeclareLaunchArgument('tool_envelope_radius', default_value='0.0'),
        DeclareLaunchArgument('tool_envelope_length', default_value='0.0'),
        DeclareLaunchArgument('table_top_z', default_value='-0.05'),
        DeclareLaunchArgument('bridge_config', default_value=share + '/config/teleop.yaml'),
        DeclareLaunchArgument('servo_config', default_value=share + '/config/servo.yaml'),
        DeclareLaunchArgument('translation_only', default_value='false',
                              description='Hold measured orientation at each grip press; ignore hand rotation'),
        DeclareLaunchArgument('fake_quest', default_value='false'),
        DeclareLaunchArgument('gripper', default_value='false', description='start the PC gripper I/O node'),
        DeclareLaunchArgument('gripper_backend', default_value='mock'),
        DeclareLaunchArgument('relay_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('relay_open_channel', default_value='2'),
        DeclareLaunchArgument('relay_close_channel', default_value='1'),
        DeclareLaunchArgument('relay_pulse_duration', default_value='0.5'),
        DeclareLaunchArgument('endpoint', default_value='true'),
        DeclareLaunchArgument('endpoint_ip', default_value='0.0.0.0',
                              description='bind address for the Quest TCP connection (all interfaces)'),
        OpaqueFunction(function=setup),
    ])
