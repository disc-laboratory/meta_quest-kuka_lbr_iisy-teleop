from glob import glob
from setuptools import setup

name = 'quest_kuka_teleop'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/config', glob('config/*')),
        ('share/' + name + '/launch', glob('launch/*.py')),
        ('share/' + name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'bridge = quest_kuka_teleop.bridge:main',
        'fake_quest = quest_kuka_teleop.fake_quest:main',
        'episode_recorder = quest_kuka_teleop.episode_recorder:main',
        'audit_episode = quest_kuka_teleop.audit_episode:main',
        'gripper_io = quest_kuka_teleop.gripper_io:main',
        'relay_gripper = quest_kuka_teleop.relay_gripper:main',
    ]},
)
