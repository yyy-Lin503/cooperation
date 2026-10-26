import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_radar_follower'

setup(
    name=package_name,
    version='8.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✨ 核心改动：自动把功能包根目录下的 launch 文件夹内所有启动文件编译到 ROS2 环境中
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TravelMate V8 tracking protocol bridge and odometry feedback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 🏎️ 注册一键双开所需的两个核心可执行节点
            'txt_radar_bridge = my_radar_follower.txt_tracker_bridge:main',
            'radar_odom_receiver = my_radar_follower.odom_receiver:main',
        ],
    },
)
