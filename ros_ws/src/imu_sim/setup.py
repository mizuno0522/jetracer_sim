from setuptools import setup

package_name = 'imu_sim'

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
    description='物理状態から 6 軸 IMU を合成する',
    license='MIT',
    entry_points={'console_scripts': ['imu_sim_node = imu_sim.imu_sim_node:main']},
)
