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
                   '--child-frame-id' , 'os_lidar'],
    )

    # base_link to zed2i_camera_center tf.
    tf_base_link_to_zed2_camera_center = Node(
        package='tf2_ros',
        name='base_link_to_zed2_camera_center',
        executable='static_transform_publisher',
        arguments=['--x'              , '0.3005',
                   '--y'              , '0',
                   '--z'              , '-0.22036',
                   '--roll'           , str(math.pi),
                   '--pitch'          , '0',
                   '--yaw'            , '0',
                   '--frame-id'       , 'base_link',
                   '--child-frame-id' , 'zed2i_camera_center'],
    )

    # base_link (NED) to seapath_frame (NED) tf.
    tf_base_link_to_seapath = Node(
        package='tf2_ros',
        name='base_link_ned_to_seapath_frame',
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

    return LaunchDescription([
        tf_base_link_to_lidar,
        tf_base_link_to_zed2_camera_center,
        tf_base_link_to_seapath,
    ])
