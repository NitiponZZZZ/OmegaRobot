from setuptools import setup
import os
from glob import glob

package_name = 'omega'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nitipon',
    maintainer_email='nitipon2544@gmail.com',
    description='DPU Robot 2022',
    license='DPU Robot 2022',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = omega.Robot_Joystick:main',
            'core = omega.Robot_Core:main',
            'rfid = omega.Robot_RFID:main',
        ],
    },
)
