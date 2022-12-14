from setuptools import setup
from glob import glob
import os

package_name = 'my_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dave',
    maintainer_email='dave@davesroboshack.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_turtlesim.my_first_program:main',
            'my_simple_publisher = my_turtlesim.my_simple_publisher:main',
            'my_simple_subscriber = my_turtlesim.my_simple_subscriber:main',
            'my_service_server = my_turtlesim.my_service_server:main',
            'my_service_client = my_turtlesim.my_service_client:main',
        ],
    },
)
