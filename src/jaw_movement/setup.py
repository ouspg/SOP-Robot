from setuptools import setup

package_name = 'jaw_movement'

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
    maintainer_email='sop-robot@example.org',
    description='ROS2 jaw movement node for speech-synchronized mouth motion.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jaw_movement_node = jaw_movement.jaw_movement_node:main'
        ],
    },
)
