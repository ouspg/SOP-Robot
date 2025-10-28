from setuptools import setup

package_name = 'unified_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SOP Robot Team',
    maintainer_email='sop@robot.local',
    description='Unified robot control node for coordinating all robot behaviors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start = unified_robot_control.unified_robot_control_node:main',
        ],
    },
)
