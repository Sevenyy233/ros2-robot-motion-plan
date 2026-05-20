from setuptools import find_packages, setup
from glob import glob

package_name = 'local_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob("launch/*.launch.py")),
        ('share/' + package_name + '/config', glob("config/*.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yangyang',
    maintainer_email='2996934032@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'local_planner_node = local_path_planner.local_planner_node:main',
            'fake_lidar_points_pub = local_path_planner.fake_lidar_points_pub:main',
        ],
    },
)
