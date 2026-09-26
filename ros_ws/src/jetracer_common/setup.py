from setuptools import setup

package_name = 'jetracer_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hiroshi Mizuno',
    maintainer_email='mizuno0522@gmail.com',
    description='ROS 非依存の共有モジュール (カメラ幾何 / アクチュエータ / IMU モデル / 参照線)',
    license='MIT',
    tests_require=['pytest'],
)
