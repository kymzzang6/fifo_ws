import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 모델 경로 설정 (사용자 환경에 맞게 수정 필수!)
    # 예: /home/ym/fifo_ws/src/ppe_detector/models/yolo26n_model
    base_model_dir = '/home/ym/fifo_ws/src/ppe_detector/models'
    
    # Launch Arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolo11n_model/merged_9',
        description='Folder name of the custom YOLO model'
    )
    
    pose_model_path_arg = DeclareLaunchArgument(
        'pose_model_path',
        default_value='yolo11n-pose.pt',
        description='Path or name of the YOLO Pose model'
    )

    # 2. PPE Detector Node (YoloDiagnostic)
    ppe_node = Node(
        package='ppe_detector',
        executable='ppe_node',
        name='ppe_node',
        output='screen',
        parameters=[{
            # 경로 조립: base_dir + / + model_name + /weights/best.pt
            'model_path': [base_model_dir, '/', LaunchConfiguration('model_name'), '/weights/best.pt'],
            'conf_thres': 0.5, # 신뢰도 임계값
            'device': 'cuda'   # GPU 강제
        }]
    )

    # 3. Pose Detector Node
    pose_node = Node(
        package='ppe_detector',
        executable='pose_node',
        name='pose_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('pose_model_path'),
            'device': 'cuda'
        }]
    )

    # 4. Decision & Visualization Node
    decision_node = Node(
        package='ppe_detector',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'check_list': ['helmet', 'vest', 'gloves', 'earplug', 'hand']
        }]
    )

    # 5. USB Camera Node (최적화 설정 적용됨)
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'framerate': 15.0,        # [최적화] 30fps -> 15fps
            'image_width': 320,       # [최적화] 640 -> 320
            'image_height': 240,
            'pixel_format': 'mjpeg2rgb', # 대역폭 절약
            'io_method': 'mmap'
        }]
    )

    return LaunchDescription([
        model_name_arg,
        pose_model_path_arg,
        usb_cam_node,
        ppe_node,
        pose_node,
        decision_node
    ])
