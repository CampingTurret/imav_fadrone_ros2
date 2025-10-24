from setuptools import find_packages, setup

package_name = 'servo_control'

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
    maintainer='elijah',
    maintainer_email='elijah.anghw@gmail.com',
    description='Driver to control servo motor',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_servo = servo_control.sim_servo:main',
            'servo_test = servo_control.servo_test:main'
        ],
    },
)
