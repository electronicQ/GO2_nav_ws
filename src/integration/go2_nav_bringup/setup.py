from setuptools import setup

package_name = 'go2_nav_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mapping.launch.py',
            'launch/localization_nav.launch.py',
        ]),
        ('share/' + package_name + '/config/rviz', [
            'config/rviz/mapping.rviz',
            'config/rviz/localization_nav.rviz',
            'config/rviz/localization_nav_light.rviz',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Integrated GO2 Voxel-SLAM and Nav2 bringup',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
