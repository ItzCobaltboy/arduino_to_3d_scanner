from setuptools import find_packages, setup

package_name = 'arduino_handler'

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
    maintainer='cobaltboy',
    maintainer_email='pathanaffan62@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_handler = arduino_handler.pyserial_handler:main',
            'data_handler = arduino_handler.data_processor:main'
        ],
    },
)
