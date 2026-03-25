from setuptools import setup

package_name = 'hand_gestures'

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
    description='ROS2 package for hand gesture command handling.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_gestures_node = hand_gestures.hand_gestures_node:main'
        ],
    },
)
