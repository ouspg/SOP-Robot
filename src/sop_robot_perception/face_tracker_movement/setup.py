from setuptools import find_packages, setup

package_name = 'face_tracker_movement'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SOP Robot Team',
    maintainer_email='sop-robot@example.org',
    description='ROS2 node that converts tracked faces into head and eye motion goals.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'face_tracker_movement_node = face_tracker_movement.face_tracker_movement_node:main'
        ],
    },
)
