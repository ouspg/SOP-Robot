from setuptools import setup
from glob import glob
import os

package_name = os.path.realpath(__file__).split(os.sep)[-2] # get name of this package
#package_name = 'emotion_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    # put python dependencies here? 
    install_requires=[
        'setuptools'
        #'numpy==1.21.3',
        #'opencv-contrib-python-headless==4.6.0.66'
    ],
    zip_safe=True,
    maintainer='vagrant',
    maintainer_email='vagrant@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection = ' + package_name + '.face_detection:main',
            'emotion_detection = ' + package_name + '.emotion_detection:main',
            'action_client = ' + package_name + '.action_client:main',
        ],
    },
)
