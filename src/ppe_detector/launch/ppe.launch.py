import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 모델 기본값 설정
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='origin1',
        description='Name of the custom YOLO model folder'
    )
    
    pose_model_path_arg = DeclareLaunchArgument(
        'pose_model_path',
        default_value='yolo11n-pose.pt',
        description='Path or name of the YOLO Pose model'
    )

    # [중요] 절대 경로 생성을 위한 기본 경로 설정 (사용자 환경에 맞게 수정 필요!)
    # 만약 소스 코드 내에 모델이 있다면 아래 경로를 확인하세요.
    base_model_dir = '/home/ym/fifo_ws/src/ppe_detector/models/yolo11n_model'

    ppe_node = Node(
        package='ppe_detector',
        executable='ppe_node',
        name='ppe_node',
        output='screen',
        parameters=[{
            # 파이썬 문자열 formatting을 사용하여 경로 생성 (PathJoinSubstitution보다 직관적)
            'model_path': [base_model_dir, '/', LaunchConfiguration('model_name'), '/weights/best.pt'],
            'conf_thres': 0.1
        }]
    )

    pose_node = Node(
        package='ppe_detector',
        executable='pose_node',
        name='pose_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('pose_model_path')
        }]
    )

    decision_node = Node(
        package='ppe_detector',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[{
            'check_list': ['helmet', 'vest', 'gloves', 'earplug']
        }]
    )

    # [수정됨] pixel_format 추가
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
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
