from setuptools import find_packages, setup

package_name = 'more_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','PCANBasic'],
    zip_safe=True,
    maintainer='Nikolajs',
    maintainer_email='n.maslovs@student.han.nl',
    description='Exports CAN messages to ROS',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = more_can.publisher_member_function:main',
                'listener = more_can.subscriber_member_function:main',
        ],
    },
)
