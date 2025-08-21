from setuptools import find_packages, setup

package_name = 'turtle_sys'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/turtle_sys.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olegk',
    maintainer_email='oleg.kirovskii@tii.ae',
    description='A ROS2 package for turtle collision avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed = turtle_sys.velocity_controller:main',
            'ttc = turtle_sys.time_to_collision:main',
        ],
    },
)
