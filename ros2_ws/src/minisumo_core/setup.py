from setuptools import setup

package_name = 'minisumo_core'

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
    maintainer='dae',
    maintainer_email='dae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logger_csv = minisumo_core.logger_csv:main',
            'odometria = minisumo_core.odometria:main',
            'teleop = minisumo_core.teleop_rutinas:main',
            'guante_node = minisumo_core.guante_control_node:main'
        ],
    },
)
