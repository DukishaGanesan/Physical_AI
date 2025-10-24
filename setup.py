from setuptools import find_packages, setup

<<<<<<< HEAD
package_name = 'temp_monitor'
=======
package_name = 'urdf_demo'
>>>>>>> fc0d916 (URDF commit)

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
<<<<<<< HEAD
    maintainer='dukisha',
    maintainer_email='dukisha@todo.todo',
=======
    maintainer='DukishaGanesan',
    maintainer_email='dukisha995@gmail.com',
>>>>>>> fc0d916 (URDF commit)
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'temperature_sensor = temp_monitor.temperature_sensor:main',
            'temp_warning = temp_monitor.temp_warning:main',
=======
>>>>>>> fc0d916 (URDF commit)
        ],
    },
)
