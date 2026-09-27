"""Camera-only RGB/IR capture session: no depth preview or motor nodes.

IR starts only while the paired recorder subscribes. Exclude depth CameraInfo
from the bridge too: that subscription alone can keep depth streaming.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sdk = '/home/robot/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin'
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', [sdk, ':', EnvironmentVariable('LD_LIBRARY_PATH',default_value='')]),
        SetEnvironmentVariable('OPENNI2_DRIVERS_PATH', sdk+'/OpenNI2/Drivers'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('openni2_camera'),'launch','camera_only.launch.py'])),
            launch_arguments={'namespace':'camera','tf_prefix':''}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('roam_e'),'launch','astra_camera.launch.py'])),
            launch_arguments={'start_depth':'false','use_foxglove':'false'}.items()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('foxglove_bridge'),'launch','foxglove_bridge_launch.xml'])),
            launch_arguments={
                'port':'8765','address':'127.0.0.1',
                'topic_whitelist':"['^/camera/color/image_raw/compressed$', '^/camera/color/image_rect/compressed$', '^/camera/color/camera_info$', '^/tf$', '^/tf_static$']",
                'param_whitelist':"['^$']",'service_whitelist':"['^$']",
                'client_topic_whitelist':"['^$']",'capabilities':'[connectionGraph]',
                'send_buffer_limit':'2000000','message_backlog_size':'64','sysinfo':'false',
            }.items()),
    ])
