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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='watimake',
    maintainer_email='elilgrade8@gmail.com',
    description='Forward kinematics for the Quanser QArm (4-DOF RPPR manipulator).',
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
        ],
    },
)
