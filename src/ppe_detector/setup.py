from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ppe_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ym',
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
            'ppe_node = ppe_detector.ppe_detector:main',
            'pose_node = ppe_detector.pose_detector:main',
            'decision_node = ppe_detector.decision_maker:main',
            
        ],
    },
)
