from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'pacr_solutions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['pacr_solutions.rrt_star'],  # <-- Add this line
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monkade1u',
    maintainer_email='mouad.monkade2@etu.univ-lorraine.fr',
    description='TODO: Package description',
    license='TODO: License declaration',

    entry_points={
        'console_scripts': [
            'path_planning = pacr_solutions.path_planning:main',
            'controller_node = pacr_solutions.controller_node:main',
            'task_planner_node = pacr_solutions.task_planner_node:main',
            'MDP = pacr_solutions.MDP:main',
        ],
    },
)
