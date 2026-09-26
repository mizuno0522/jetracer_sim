from setuptools import setup

package_name = 'jetracer_compat'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Hiroshi Mizuno', maintainer_email='mizuno0522@gmail.com',
    description='JetRacer (NvidiaRacecar / jetcam CSICamera) compatible classes backed by the sim',
    license='MIT',
    entry_points={'console_scripts': ['jetracer_compat_demo = jetracer_compat.demo:main']},
)
