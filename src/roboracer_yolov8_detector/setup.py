from setuptools import find_packages, setup

package_name = 'roboracer_yolov8_detector'

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
    maintainer='loq',
    maintainer_email='ahmaddurranitrg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'yolov8_detection_node = roboracer_yolov8_detector.yolov8_detection_node:main', 
          'detection_3d_node = roboracer_yolov8_detector.detection_3d_node:main', 
        ],
    },
)
