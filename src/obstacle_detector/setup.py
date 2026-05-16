from setuptools import find_packages, setup

package_name = 'obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caps',
    maintainer_email='kymzzang6@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
            
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = obstacle_detector.obstacle_detector:main',
            'obstacle_debug_node = obstacle_detector.obstacle_detector:main',
            'video_streaming_node = obstacle_detector.videostreaming_node:main',
        ],
    },
)
