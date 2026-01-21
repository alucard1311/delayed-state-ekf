from setuptools import setup

package_name = 'auv_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mission.yaml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Mission planner and state machine for AUV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = auv_planner.planner_node:main',
        ],
    },
)
