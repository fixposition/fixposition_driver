from setuptools import find_packages, setup

package_name = 'fixposition_tf_publisher_ros2'

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
    maintainer='boson',
    maintainer_email='ramanab@bosonmotors.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fixposition_tf_publisher = fixposition_tf_publisher_ros2.fixposition_tf_publisher:main'
        ],
    },
)
