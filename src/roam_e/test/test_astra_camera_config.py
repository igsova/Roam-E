from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CAMERA_LAUNCH = PACKAGE_ROOT / 'launch' / 'astra_camera.launch.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
SETUP_FILE = PACKAGE_ROOT / 'setup.py'
BOOTSTRAP = PACKAGE_ROOT.parents[1] / 'scripts' / 'setup' / 'bootstrap_pi5.sh'


def test_astra_launch_uses_verified_depth_and_color_drivers():
    source = CAMERA_LAUNCH.read_text(encoding='utf-8')

    # Driver composed directly (not openni2's camera_only.launch.py, whose
    # tfs.launch.py publishes "/camera_link"-style frames Foxglove cannot join).
    assert "plugin='openni2_wrapper::OpenNI2Driver'" in source
    assert "'camera_only.launch.py'" not in source
    for frame in ('camera_depth_frame', 'camera_rgb_frame',
                  'camera_depth_optical_frame', 'camera_rgb_optical_frame'):
        assert f"'--child-frame-id', '{frame}'" in source
    assert "'/camera" not in source.replace("'/camera/", "")
    assert "executable='mjpeg_camera_node'" in source
    assert "'OPENNI2_DRIVERS_PATH'" in source
    assert "'LD_LIBRARY_PATH'" in source
    assert "'capture_fps': 30.0" in source
    assert "'color_width', default_value='640'" in source
    assert "'color_height', default_value='480'" in source
    assert "'color_preview_fps', default_value='5.0'" in source
    assert "'color_decode_fps', default_value='2.0'" in source
    assert "'usb_payload_bytes': 800" in source
    assert "namespace='camera/color'" in source
    assert "'camera_frame_id': 'camera_rgb_optical_frame'" in source
    assert "FindPackageShare('foxglove_bridge')" in source
    assert "'use_foxglove'" in source
    assert "default_value='8765'" in source
    assert "'address': '127.0.0.1'" in source
    assert "'^/camera/color/image_raw/compressed$'" in source
    assert "'^/camera/color/image_rect/compressed$'" in source
    assert "'color_calibration_file'" in source
    assert "'astra_rgb_640x480.yaml'" in source
    assert "'rectify_color', default_value='true'" in source
    assert "package='image_proc', executable='rectify_node'" in source
    assert "('image', 'image_raw')" in source
    assert "executable='depth_preview_node'" in source
    assert "'input_topic': '/camera/depth_raw/image'" in source
    assert "'output_topic': '/camera/depth_preview/compressed'" in source
    assert "'publish_rate': 10.0" in source
    assert "'downsample': 2" in source
    assert "'^/camera/depth_preview/compressed$'" in source
    assert "'param_whitelist': \"['^$']\"" in source
    assert "'service_whitelist': \"['^$']\"" in source
    assert "'client_topic_whitelist': \"['^$']\"" in source
    assert "'capabilities': '[connectionGraph]'" in source
    assert "'send_buffer_limit': '2000000'" in source
    assert "'sysinfo': 'false'" in source


def test_astra_launch_is_installed_with_runtime_dependencies():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')
    package_source = PACKAGE_XML.read_text(encoding='utf-8')

    assert "'launch/astra_camera.launch.py'" in setup_source
    assert '<exec_depend>openni2_camera</exec_depend>' in package_source
    assert '<exec_depend>v4l2_camera</exec_depend>' in package_source
    assert (
        '<exec_depend>compressed_image_transport</exec_depend>'
        in package_source
    )
    assert (
        '<exec_depend>compressed_depth_image_transport</exec_depend>'
        in package_source
    )
    assert '<exec_depend>python3-numpy</exec_depend>' in package_source
    assert '<exec_depend>python3-opencv</exec_depend>' in package_source
    assert '<exec_depend>python3-yaml</exec_depend>' in package_source
    assert '<exec_depend>rcl_interfaces</exec_depend>' in package_source
    assert '<exec_depend>image_proc</exec_depend>' in package_source
    assert "'config/astra_rgb_640x480.yaml'" in setup_source
    assert 'depth_preview_node = roam_e.depth_preview_node:main' in setup_source


def test_pi5_bootstrap_grants_scoped_astra_access():
    source = BOOTSTRAP.read_text(encoding='utf-8')

    assert 'ATTR{idVendor}=="2bc5"' in source
    assert 'ATTR{idProduct}=="0403"' in source
    assert 'GROUP:="video"' in source
    assert 'MODE:="0660"' in source
    assert 'libOpenNI2.so.0' in source
