import math

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link to os_lidar tf.
    tf_base_link_to_lidar = Node(
        package='tf2_ros',
        name='base_link_to_lidar',
        executable='static_transform_publisher',
        arguments=['--x'              , '-0.0145',
                   '--y'              , '0',
                   '--z'              , '-0.392425',
                   '--roll'           , str(math.pi),
                   '--pitch'          , '0',
                   '--yaw'            , '0',
                   '--frame-id'       , 'base_link',
                   '--child-frame-id' , 'os_sensor'],
    )

    # base_link to zed_camera_link tf.
    tf_base_link_to_zed_camera_link = Node(
        package='tf2_ros',
        name='base_link_to_zed_camera_link',
        executable='static_transform_publisher',
        arguments=['--x'              , '0.3005',
                   '--y'              , '0',
                   '--z'              , '-0.22036',
                   '--roll'           , str(math.pi),
                   '--pitch'          , '0',
                   '--yaw'            , '0',
                   '--frame-id'       , 'base_link',
                   '--child-frame-id' , 'zed_camera_link'],
    )

    # base_link (NED) to seapath_frame (NED) tf.
    tf_seapath_to_base_link = Node(
        package='tf2_ros',
        name='seapath_to_base_link',
        executable='static_transform_publisher',
        arguments=['--x'              , '0',
                   '--y'              , '0',
                   '--z'              , '0',
                   '--roll'           , '0',
                   '--pitch'          , '0',
                   '--yaw'            , '0',
                   '--frame-id'       , 'seapath',
                   '--child-frame-id' , 'base_link'],
    )

    os_sensor_to_os_lidar = Node(
        package='tf2_ros',
        name='os_sensor_to_os_lidar',
        executable='static_transform_publisher',
        arguments=['--x'              , '0',
                   '--y'              , '0',
                   '--z'              , '0.036180000000000004',
                   '--roll'           , '0',
                   '--pitch'          , '0',
                   '--yaw'            , str(math.pi),
                   '--frame-id'       , 'os_sensor',
                   '--child-frame-id' , 'os_lidar'],
    )

    return LaunchDescription([
        tf_base_link_to_lidar,
        tf_base_link_to_zed_camera_link,
        tf_seapath_to_base_link,
        os_sensor_to_os_lidar,
    ])
