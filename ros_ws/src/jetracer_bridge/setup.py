import os
from glob import glob
from setuptools import setup

package_name = 'jetracer_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hiroshi Mizuno',
    maintainer_email='mizuno0522@gmail.com',
    description='JetRacer 実機ブリッジ (PCA9685 / MPU-6500 / CSI)',
    license='MIT',
    entry_points={'console_scripts': [
        'jetracer_bridge = jetracer_bridge.bridge_node:main',
        'csi_camera_node = jetracer_bridge.csi_camera_node:main',
        'imu_probe = jetracer_bridge.mpu6500:main',
    ]},
)
