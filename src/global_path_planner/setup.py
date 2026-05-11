from setuptools import find_packages, setup

package_name = 'global_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
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
            'planner_node = global_path_planner.planner_node:main',
        ],
    },
)
