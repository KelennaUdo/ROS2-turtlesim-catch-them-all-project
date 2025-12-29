from setuptools import find_packages, setup

package_name = 'turtlesim_main_py_pkg'

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
    maintainer='kelenna-udo',
    maintainer_email='udo.kelenna.uma.15@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "turtle_controller = turtlesim_main_py_pkg.turtle_controller:main",
            "turtle_spawner = turtlesim_main_py_pkg.turtle_spawner:main",
            "turtlesim_node = turtlesim_main_py_pkg.turtlesim_node:main"
        ],
    },
)
