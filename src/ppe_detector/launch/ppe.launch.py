import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 모델 경로 설정 (사용자 환경에 맞게 수정 필수!)
    # 예: /home/ym/fifo_ws/src/ppe_detector/models/yolo26n_model
    base_model_dir = '/home/caps/fifo_ws/src/ppe_detector/models'
    
    # Launch Arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolo26n_model/origin3_addhand3',
        description='Folder name of the custom YOLO model'
    )
    
    pose_model_path_arg = DeclareLaunchArgument(
        'pose_model_path',
        default_value='yolo11n-pose.pt',
        description='Path or name of the YOLO Pose model'
    )

    # 2. PPE Detector Node (YoloDiagnostic)
    total_thermo_node = Node(
        package='ppe_detector',
        executable='total_thermo_node',
        name='total_thermo_node',
        output='screen',
        parameters=[{
            # 경로 조립: base_dir + / + model_name + /weights/best.pt
            'model_path': [base_model_dir, '/', LaunchConfiguration('model_name'), '/weights/best.engine'],
            'conf_thres': 0.75, # 신뢰도 임계값
            'device': 'cuda'   # GPU 강제
        }]
    )

    # 3. Pose Detector Node
    thermo_pub_node = Node(
        package='thermoeye_driver',
        executable='thermo_pub_node',
        name='thermo_pub_node',
        output='screen',
       
    )

    
    

    return LaunchDescription([
        model_name_arg,
        pose_model_path_arg,
        thermo_pub_node,
        total_thermo_node,
    ])
