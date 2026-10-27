"""KUKA iiQKA EAC driver for the LBR iisy 3 R760. Starts UNCONFIGURED: nothing moves
until robot_manager is configured+activated (scripts/robot_lifecycle.sh).
mode is required: mock | hardware. Fixed addresses: PC 192.168.44.2, KONI 192.168.44.1."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    eac = get_package_share_directory('kuka_iiqka_eac_driver') + '/launch/startup.launch.py'
    a = lambda n: LaunchConfiguration(n)
    return LaunchDescription([
        DeclareLaunchArgument('mode', description='mock | hardware (no default on purpose)'),
        DeclareLaunchArgument('rt_prio', default_value='70'),
        DeclareLaunchArgument('lock_memory', default_value='true'),
        IncludeLaunchDescription(eac, launch_arguments={
            'robot_model': 'lbr_iisy3_r760', 'mode': a('mode'),
            'controller_ip': '192.168.44.1', 'client_ip': '192.168.44.2',
            'rt_prio': a('rt_prio'), 'lock_memory': a('lock_memory')}.items()),
    ])
