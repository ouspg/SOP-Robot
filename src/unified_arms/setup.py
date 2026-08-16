from setuptools import setup

package_name = 'unified_arms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vagrant',
    maintainer_email='vagrant@todo.todo',
    description='Coordinate unified arm and hand gestures.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'unified_arms_node = unified_arms.unified_arms_node:main',
        ],
    },
)
