# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():


    from_bag_arg = DeclareLaunchArgument(
        'from_bag', default_value='False',
        description='Whether to run from a bag or live realsense data')
    
    """Launch file which brings up visual slam node configured for RealSense."""
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
        }],
        condition=IfCondition(LaunchConfiguration('from_bag'))
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{
            'size': 0.08,
        }],
        remappings=[('/image', 'camera/color/image_raw'),
                    ('/camera_info','camera/color/camera_info')],
        condition=IfCondition(LaunchConfiguration('from_bag'))
    )

    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'json_file_path': '/workspaces/isaac_ros-dev/launch/realsense_presets/highaccuracy.json',
                'color_height': 1080,
                'color_width': 1920,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2
        }],
        condition=UnlessCondition(LaunchConfiguration('from_bag'))
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 22.00
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
        ],
        output='screen'
    )

    master = Node(
        package='slam_master',
        executable='master_node',
        name='master_node',
        output='screen'
    )

    # Rviz
    config_path = '/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/realsense.custom.rviz'
    print(config_path)
    # Rviz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_path],    # set the config
        output='screen')


    return launch.LaunchDescription([from_bag_arg, master, rviz, visual_slam_launch_container, realsense_camera_node, apriltag_container])
