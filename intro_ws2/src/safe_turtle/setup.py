from setuptools import find_packages, setup

package_name = 'safe_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olegkirovskii',
    maintainer_email='oleg.kirovskii@tii.ae',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'border = safe_turtle.border_watch:main',
            'ttc = safe_turtle.time_to_collision:main',
            'speed = safe_turtle.speed_commander:main',
        ],
    },
)
