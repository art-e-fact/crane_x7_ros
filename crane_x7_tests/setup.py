from setuptools import find_packages, setup

package_name = 'crane_x7_tests'

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
    maintainer='decarabas',
    maintainer_email='codrea.eric@gmail.com',
    description='CRANE-X7 example test',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_pose = crane_x7_tests.joint_state_subscriber:main",
            "arm_pose = crane_x7_tests.arm_pose_subscriber:main"

        ],
    },
)

