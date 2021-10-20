from setuptools import setup

package_name = 'my_tfg_pkg'

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
    maintainer='pparrilla',
    maintainer_email='pedroparrilla@pm.me',
    description='Sistema de comunicacion e interaccion de robots en la agricultura',
    license='GNU GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_node = my_tfg_pkg.sensor_node:main",
            "controller_node = my_tfg_pkg.controller_node:main",
            "heater_node = my_tfg_pkg.heater_node:main",
            "window_node = my_tfg_pkg.window_node:main",
            "irradiance_publisher = my_tfg_pkg.irradiance_publisher:main",
            "temp_hum_publisher = my_tfg_pkg.temp_hum_publisher:main",
            "firebase_service = my_tfg_pkg.firebase_service:main"
        ],
    },
)
