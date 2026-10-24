from setuptools import setup
from glob import glob
package_name = 'agt_robot_studio'
setup(
    name=package_name, version='0.2.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                ('share/'+package_name, ['package.xml']),
                ('share/'+package_name+'/config', glob('config/*.yaml'))],
    install_requires=['setuptools', 'numpy'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AGT Robotics', maintainer_email='noreply@example.com',
    description='Safe ROS 2 desktop studio with live GLIM point cloud and mapping/test tools',
    license='Apache-2.0',
    entry_points={'console_scripts': ['agt_robot_studio = agt_robot_studio.ui:main']},
)
