from glob import glob
from setuptools import find_packages, setup

package_name = 'panda_planning'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name, ['package.xml']),
        ('share/'+package_name+'/launch', glob('launch/*.launch.py')),
        ('share/'+package_name+'/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools', 'numpy', 'pybullet', 'scipy'],
    zip_safe=True,
    maintainer='jasonz',
    maintainer_email='zhenyuzhang763@gmail.com',
    description='Panda static-scene collision checking, RRT and quintic trajectories',
    license='UNLICENSED',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'planner_node = panda_planning.planner_node:main',
        'control_node = panda_planning.control_node:main',
        'startup = panda_planning.startup:main',
    ]},
)
