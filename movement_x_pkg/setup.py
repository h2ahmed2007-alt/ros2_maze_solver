from setuptools import find_packages, setup

package_name = 'movement_x_pkg'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                    'total_move = movement_x_pkg.total_move:main',
                    'pid_controller = movement_x_pkg.pid_controller:main',
                    'pid_servers = movement_x_pkg.pid_servers:main',
                    'pid_subs = movement_x_pkg.pid_subs:main',
        ],
    },
)
