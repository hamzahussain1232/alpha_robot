from setuptools import setup, find_packages

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    package_dir={'': 'src'},  # if you move code to src/articubot_one
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'serial_diffdrive_node = articubot_one.serial_diffdrive_node:main',
            'keyboard_bridge = articubot_one.keyboard_bridge:main',
            'voice_command_receiver_node = articubot_one.voice_command_receiver_node:main',
        ],
    },
)