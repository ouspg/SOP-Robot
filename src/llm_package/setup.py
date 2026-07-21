from setuptools import setup

package_name = 'llm_package'

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
    maintainer='Aapo Pihlajaniemi',
    maintainer_email='apihlaja20@student.oulu.fi',
    description='Local Finnish conversational language model.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'llm_node = llm_package.llm_node:main',
        ],
    },
)
