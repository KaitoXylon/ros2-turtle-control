from setuptools import setup
import os
from glob import glob

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashra',
    maintainer_email='ashra@todo.todo',
    description='Turtle control package with figure-eight driver and pen toggle service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'figure8_driver = turtle_control.figure8_driver:main',
            'trace_toggle = turtle_control.trace_toggle:main',  # <-- Added a comma here
            'turtle_circle = turtle_control.turtle_circle:main',
            'turtle_square = turtle_control.turtle_square:main',
            'autoturtle = turtle_control.autoturtle:main',
            'automoveinput = turtle_control.automoveinput:main',
            'turtle = turtle_control.turtle:main',
        ],
    },
)
