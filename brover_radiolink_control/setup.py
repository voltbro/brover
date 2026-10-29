from setuptools import find_packages, setup

package_name = 'brover_radiolink_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='sashavera1998@gmail.com',
    description='Translate Radiolink joystick input to cmd_vel.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radiolink_node = brover_radiolink_control.radiolink_node:main',
            'direct_joystick_node = '
            'brover_radiolink_control.direct_joystick_node:main',
        ],
    },
)
