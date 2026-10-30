from setuptools import setup

package_name = 'qarm_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_to.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='watimake',
    maintainer_email='elilgrade8@gmail.com',
    description='Forward and inverse kinematics for the Quanser QArm (4-DOF RPPR manipulator).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hand sanity-check: pass four joint angles in DEGREES, get a pose.
            #   ros2 run qarm_kinematics qarm_fk 0 0 0 0
            'qarm_fk = qarm_kinematics.forward_kinematics:main',
            # Headless FK-vs-TF comparison table (see validate_fk_vs_tf.py).
            #   ros2 run qarm_kinematics qarm_validate_fk
            'qarm_validate_fk = qarm_kinematics.validate_fk_vs_tf:main',
            # Target position -> joint angles (see inverse_kinematics.py).
            #   ros2 run qarm_kinematics qarm_ik 0.3 0.1 0.4 [WRIST] [--tip]
            'qarm_ik = qarm_kinematics.inverse_kinematics:main',
            # IK PASS/FAIL validation table (see validate_ik.py).
            #   ros2 run qarm_kinematics qarm_validate_ik
            'qarm_validate_ik = qarm_kinematics.validate_ik:main',
            # Drive the arm to a target point via IK (see move_to_node.py).
            #   ros2 launch qarm_kinematics move_to.launch.py
            'qarm_move_to = qarm_kinematics.move_to_node:main',
        ],
    },
)
