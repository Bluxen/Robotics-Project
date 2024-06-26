from setuptools import setup
from glob import glob
package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=['rm'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ## NOTE: you must add this line to use launch files
        # Instruct colcon to copy launch files during package build 
        ('share/' + package_name + '/launch', glob('launch/*.launch.*'))
    ],
    install_requires=['setuptools', 'cv_bridge'],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='robotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate = project.calibrate:main',
            'vs = rm.main:main'
        ],
    },
)
