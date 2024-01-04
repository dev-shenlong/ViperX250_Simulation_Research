from setuptools import find_packages, setup

package_name = 'interbotix_vx250_sim'

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
    maintainer='shenlong',
    maintainer_email='dev.shenlong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    "vx250_custom_controller = interbotix_vx250_sim.vx250_gazebo_controller:main"
        ],
    },
)
