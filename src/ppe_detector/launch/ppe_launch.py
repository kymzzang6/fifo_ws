import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. Launch Arguments (파라미터 설정) ---
    # YOLO 모델 경로
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/yolo11n_model/merged_8/weights/best.pt',
        description='Absolute path to the custom YOLO model file'
    )
    
    # Pose 모델 경로
    pose_model_path_arg = DeclareLaunchArgument(
        'pose_model_path',
        default_value='yolo11n-pose.pt',
        description='Path or name of the YOLO Pose model'
    )



    # --- 2. Node 정의 ---
    
    # (1) YOLO Detector (PPE 탐지)
    ppe_node = Node(
        package='ppe_detector',    # 실제 패키지명 입력 (예: ppe_detector)
        executable='ppe_node',     # 실제 실행파일 이름 (setup.py의 entry_points 이름)
        name='ppe_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_thres': 0.1
        }]
    )

    # (2) Pose Detector (사람 관절 탐지)
    pose_node = Node(
        package='ppe_detector',    # 실제 패키지명 입력
        executable='pose_node',     # 실제 실행파일 이름
        name='pose_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('pose_model_path')
        }]
    )

    # (3) PPE Matcher (매칭 및 판단)
    decision_node = Node(
        package='ppe_detector',    # 실제 패키지명 입력
        executable='decision_node',  # 실제 실행파일 이름
        name='decision_node',
        output='screen',
        parameters=[{
            'check_list': ['helmet', 'vest', 'gloves', 'earplug']
        }]
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{'video_device': '/dev/video4'}]
    )

    return LaunchDescription([
        model_path_arg,
        pose_model_path_arg,
        usb_cam_node,
        ppe_node,
        pose_node,
        decision_node
    ])
