from setuptools import find_packages, setup
package_name = 'agt_dynamic_map_cleaner'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/start_clearance.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT Field Test', maintainer_email='field-test@example.invalid',
    description='Conservative offline post-mapping free-space evidence cleaner',
    license='Apache-2.0',
    entry_points={'console_scripts': ['clean_start = agt_dynamic_map_cleaner.cli:main']},
)
