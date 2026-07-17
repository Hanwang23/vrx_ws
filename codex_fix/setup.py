from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'codex_usv_controller'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        *[
            (
                os.path.join('share', package_name, os.path.dirname(model_file)),
                [model_file],
            )
            for model_file in glob('models/*/*')
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='han@example.com',
    description='Closed-loop autonomous controller for the VRX WAM-V.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_usv = codex_usv_controller.node:main',
            'gnss_odometry_adapter = codex_usv_controller.gnss_odometry:main',
            'fit_3dof_model = codex_usv_controller.system_identification:main',
            'moving_target = codex_usv_controller.moving_target:main',
            'regression_monitor = codex_usv_controller.regression_monitor:main',
            'run_evaluation = codex_usv_controller.evaluation_runner:main',
        ],
    },
)
