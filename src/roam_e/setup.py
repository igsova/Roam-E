from setuptools import find_packages, setup

package_name = 'roam_e'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/sensors_viz.launch.py',
                'launch/roam_e_imu.launch.py',
                'launch/drive.launch.py',
                'launch/mapping.launch.py',
                'launch/rtabmap_mapping.launch.py',
                'launch/localization.launch.py',
                'launch/navigation.launch.py',
                'launch/astra_camera.launch.py',
            ],
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/ekf.yaml',
                'config/slam.yaml',
                'config/amcl.yaml',
                'config/nav2.yaml',
                'config/astra_rgb_640x480.yaml',
            ],
        ),
        (
            'share/' + package_name + '/behavior_trees',
            ['behavior_trees/navigate_to_pose_bounded.xml'],
        ),
        ('share/' + package_name + '/rviz', ['rviz/sensors_viz.rviz']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'pyserial',
        'rplidar',
        'adafruit-blinka',
        'adafruit-circuitpython-bno055',
    ],
    zip_safe=True,
    maintainer='you',
    maintainer_email='igorsouzavazz@gmail.com',
    description=(
        'Sensor visualization, wheel odometry, and guarded differential '
        'drive control for Roam-E'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_localization_node = '
            'roam_e.auto_localization_node:main',
            'bno055_imu_node = roam_e.imu_bno055_node:main',
            'depth_preview_node = roam_e.depth_preview_node:main',
            'depth_obstacle_cloud_node = '
            'roam_e.depth_obstacle_cloud_node:main',
            'mjpeg_camera_node = roam_e.mjpeg_camera_node:main',
            'encoder_odometry_node = roam_e.encoder_odometry_node:main',
            'goal_pose_bridge = roam_e.goal_pose_bridge:main',
            'rplidar_a1_node = roam_e.rplidar_a1_node:main',
        ],
    },
)
