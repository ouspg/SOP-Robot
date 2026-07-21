from glob import glob

from setuptools import setup

package_name = 'full_demo'

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
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vagrant',
    maintainer_email='vagrant@todo.todo',
    description='Launch and coordinate the complete robot demo.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'full_demo_node = full_demo.full_demo_node:main',
        ],
    },
)
