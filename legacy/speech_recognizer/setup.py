from setuptools import setup

package_name = 'speech_recognizer'

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
    maintainer='Juha-Matti Runtti',
    maintainer_email='jruntti20@student.oulu.fi',
    description='ROS2 speech recognition node backed by SpeechRecognition and Whisper.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['speech_recognizer_node = speech_recognizer.speech_recognizer:main'],
    },
)
