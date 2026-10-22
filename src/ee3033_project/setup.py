from glob import glob
from setuptools import find_packages, setup

setup(
    name='ee3033_project', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ee3033_project']),
        ('share/ee3033_project', ['package.xml']),
        ('share/ee3033_project/launch', glob('launch/*.launch.py')),
        ('share/ee3033_project/config', glob('config/*.yaml')),
        ('share/ee3033_project/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/ee3033_project/rviz', glob('rviz/*.rviz')),
        ('share/ee3033_project/web', glob('web/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='EE3033 Project Group 4', maintainer_email='group4@example.invalid',
    description='Autonomous target search and return for the EE3033 balance car.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission = ee3033_project.mission:main',
        'perception = ee3033_project.perception:main',
        'target_filter = ee3033_project.target_filter:main',
        'velocity_gate = ee3033_project.velocity_gate:main',
        'scan_resampler = ee3033_project.scan_resampler:main',
        'console = ee3033_project.console:main',
        'mapper = ee3033_project.mapper:main',
    ]},
)
