from setuptools import find_packages, setup

package_name = 'uncertainty_navigation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='barsha',
    maintainer_email='barsha@todo.todo',

    description='Uncertainty-aware autonomous robot navigation',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'hello_robot = uncertainty_navigation.hello_robot:main',
            'lidar_subscriber = uncertainty_navigation.lidar_subscriber:main',
            'robot_controller = uncertainty_navigation.robot_controller:main',
            'uncertainty_estimator = uncertainty_navigation.uncertainty_estimator:main',
            'goal_navigator = uncertainty_navigation.goal_navigator:main',
        ],
    },
)
