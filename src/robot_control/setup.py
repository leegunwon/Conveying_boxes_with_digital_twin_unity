from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Launch 파일 경로 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunwon',
    maintainer_email='dlrjsdnjs111@tukorea.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_7 = robot_control.task_7:main',
            'simple_manager_node = robot_control.simple_manager_node:main',
        ],
    },
)
