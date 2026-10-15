from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    openni2_sdk_dir = LaunchConfiguration('openni2_sdk_dir')
    start_depth = LaunchConfiguration('start_depth')
    start_color = LaunchConfiguration('start_color')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'openni2_sdk_dir',
            default_value=EnvironmentVariable(
                'ROAM_E_OPENNI2_SDK_DIR',
                default_value=(
                    '/home/robot/'
                    'OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin'
                ),
            ),
            description='Directory containing the legacy Orbbec OpenNI2 SDK.',
        ),
        DeclareLaunchArgument(
            'start_depth',
            default_value='true',
            description='Start the OpenNI2 depth stream.',
        ),
        DeclareLaunchArgument(
            'start_color',
            default_value='true',
            description='Start the separate libuvc color stream.',
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description=(
                'Start a loopback-only, read-only Foxglove bridge for '
                'standalone camera tests.'
            ),
        ),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('color_preview_fps', default_value='5.0'),
        DeclareLaunchArgument('color_decode_fps', default_value='2.0'),
        DeclareLaunchArgument(
            'color_calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('roam_e'), 'config', 'astra_rgb_640x480.yaml']),
            description='Measured RGB-only calibration; must match capture resolution.',
        ),
        DeclareLaunchArgument('rectify_color', default_value='true',
                              description='Publish rectified RGB on demand, using image_proc.'),

        # The ROS wrapper is linked against libOpenNI2.so.0. The legacy
        # Orbbec SDK and its driver must take precedence over Ubuntu's OpenNI2
        # library, which detects no Astra Pro depth device.
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [
                openni2_sdk_dir,
                ':',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            'OPENNI2_DRIVERS_PATH',
            [openni2_sdk_dir, '/OpenNI2/Drivers'],
        ),

        # Same driver and parameters as openni2_camera/camera_only.launch.py,
        # but without its tfs.launch.py: that always prefixes frames with "/"
        # ("/camera_link"), which tf2 strips but Foxglove shows as a separate,
        # unconnected tree. The camera's static frames are published below
        # under the same names the images use (no leading slash).
        ComposableNodeContainer(
            name='container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(start_depth),
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace='camera',
                    parameters=[
                        {'depth_registration': True},
                        {'use_device_time': True},
                        {'rgb_frame_id': 'camera_rgb_optical_frame'},
                        {'depth_frame_id': 'camera_depth_optical_frame'},
                        {'ir_frame_id': 'camera_ir_optical_frame'},
                    ],
                ),
            ],
        ),
        # Upstream openni2 offsets, children of the description's camera_link
        # (the description itself excludes these optical frames by default).
        *[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=name,
                output='screen',
                condition=IfCondition(start_depth),
                arguments=arguments,
            )
            for name, arguments in (
                ('camera_depth_frame_tf', ['--frame-id', 'camera_link',
                    '--child-frame-id', 'camera_depth_frame', '--y', '-0.02']),
                ('camera_rgb_frame_tf', ['--frame-id', 'camera_link',
                    '--child-frame-id', 'camera_rgb_frame', '--y', '-0.045']),
                ('camera_depth_optical_tf', ['--frame-id', 'camera_depth_frame',
                    '--child-frame-id', 'camera_depth_optical_frame',
                    '--roll', '-1.5707963267948966', '--yaw', '-1.5707963267948966']),
                ('camera_rgb_optical_tf', ['--frame-id', 'camera_rgb_frame',
                    '--child-frame-id', 'camera_rgb_optical_frame',
                    '--roll', '-1.5707963267948966', '--yaw', '-1.5707963267948966']),
            )
        ],

        # The Astra Pro exposes RGB as a separate UVC device. OpenNI advertises
        # an RGB topic for this model but produces no frames, so publish color
        # under /camera/color to keep it distinct from that placeholder.
        Node(
            package='roam_e',
            executable='mjpeg_camera_node',
            namespace='camera/color',
            name='color_camera',
            output='screen',
            condition=IfCondition(start_color),
            parameters=[{
                # Direct libuvc capture limits USB reservation, retaining VGA
                # RGB and VGA depth without the V4L2 simultaneous-stream stall.
                'width': ParameterValue(LaunchConfiguration('color_width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('color_height'), value_type=int),
                'capture_fps': 30.0,
                'usb_payload_bytes': 800,
                'preview_fps': ParameterValue(LaunchConfiguration('color_preview_fps'), value_type=float),
                'decode_fps': ParameterValue(LaunchConfiguration('color_decode_fps'), value_type=float),
                'camera_frame_id': 'camera_rgb_optical_frame',
                'calibration_file': ParameterValue(
                    LaunchConfiguration('color_calibration_file'), value_type=str),
            }],
        ),

        # The official rectifier subscribes lazily: no decoded RGB or remapping
        # work unless an image_rect consumer (including compressed) connects.
        GroupAction(condition=IfCondition(start_color), actions=[
            Node(
                package='image_proc', executable='rectify_node',
                namespace='camera/color', name='color_rectify', output='screen',
                condition=IfCondition(LaunchConfiguration('rectify_color')),
                parameters=[{'image_transport': 'raw', 'queue_size': 5, 'interpolation': 1}],
                remappings=[('image', 'image_raw'), ('image_rect', 'image_rect')],
            ),
        ]),

        # Decimated XYZ cloud for Nav2's voxel obstacle layer (idle unless
        # subscribed). 160x120 points at <=5 Hz, 0.45-3.0 m.
        Node(
            package='roam_e',
            executable='depth_obstacle_cloud_node',
            name='depth_obstacle_cloud',
            output='screen',
            condition=IfCondition(start_depth),
            parameters=[{
                'depth_topic': '/camera/depth_raw/image',
                'info_topic': '/camera/depth_raw/camera_info',
                'output_topic': '/camera/obstacle_points',
                'publish_rate': 5.0,
                'stride': 4,
                'min_range_m': 0.45,
                'max_range_m': 3.0,
            }],
        ),

        # compressed_depth_image_transport preserves metric depth, but its
        # custom wire format is not rendered by Foxglove's Image panel. This
        # low-rate preview keeps navigation-quality raw depth intact while
        # exposing a normal JPEG that is cheap to view over the SSH tunnel.
        Node(
            package='roam_e',
            executable='depth_preview_node',
            name='depth_preview',
            output='screen',
            condition=IfCondition(start_depth),
            parameters=[{
                'input_topic': '/camera/depth_raw/image',
                'output_topic': '/camera/depth_preview/compressed',
                'min_depth_m': 0.2,
                'max_depth_m': 5.0,
                'publish_rate': 10.0,
                'downsample': 2,
                'jpeg_quality': 70,
            }],
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml',
                ])
            ),
            condition=IfCondition(use_foxglove),
            launch_arguments={
                'port': foxglove_port,
                'address': '127.0.0.1',
                'topic_whitelist': (
                    "['^/camera/color/image_raw/compressed$', "
                    "'^/camera/color/image_rect/compressed$', "
                    "'^/camera/depth_preview/compressed$', "
                    "'^/camera/color/camera_info$', "
                    "'^/camera/depth_raw/camera_info$', "
                    "'^/tf$', '^/tf_static$']"
                ),
                'param_whitelist': "['^$']",
                'service_whitelist': "['^$']",
                'client_topic_whitelist': "['^$']",
                'capabilities': '[connectionGraph]',
                'send_buffer_limit': '2000000',
                'message_backlog_size': '64',
                'sysinfo': 'false',
            }.items(),
        ),
    ])
