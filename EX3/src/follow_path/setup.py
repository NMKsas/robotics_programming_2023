from setuptools import find_packages, setup

package_name = 'follow_path'

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
    maintainer='sasnmk',
    maintainer_email='75199661+NMKsas@users.noreply.github.com',
    description='Controller for ros2 turtlebot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'path_controller = follow_path.follow_path:main'
        ],
    },
)
