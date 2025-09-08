from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_tb3_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ygnk0805@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # setup.py の中身
entry_points={
    'console_scripts': [
        'obstacle_avoider = my_tb3_controller.obstacle_avoider:main', # この行を追加！
        # もし他のファイルもあれば、カンマで区切って追記
    ],
},
)
