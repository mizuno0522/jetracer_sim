import os
from glob import glob
from setuptools import setup

package_name = 'jetracer_stack'

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
    description='推論スタックの器 (cmd_shaper / failsafe / policy_net / gt_teacher / description)',
    license='MIT',
    entry_points={'console_scripts': [
        'cmd_shaper = jetracer_stack.cmd_shaper:main',
        'failsafe = jetracer_stack.failsafe:main',
        'gt_teacher = jetracer_stack.gt_teacher:main',
        'policy_net = jetracer_stack.policy_net:main',
        'urdf_gen = jetracer_stack.urdf_gen:main',
    ]},
)
