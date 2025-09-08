from setuptools import find_packages, setup

package_name = 'sensor_sim'

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
    maintainer='root',
    maintainer_email='ygnk0805@outlook.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'sensor_pub = sensor_sim.sensor_pub:main',
        'sensor_sub = sensor_sim.sensor_sub:main',
    ],
},

)
