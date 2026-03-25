from setuptools import find_packages, setup

package_name = 'roboracer_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'visualization_msgs'],
    zip_safe=True,
    maintainer='loq',
    maintainer_email='ahmaddurranitrg@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "wall_follow = roboracer_py.wall_follow:main",
            "gap_follow = roboracer_py.gap_follow:main",    
            "disparity_extender = roboracer_py.disparity_extender:main"
        ],
    },
)
