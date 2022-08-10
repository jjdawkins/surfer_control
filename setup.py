from setuptools import setup

package_name = 'surfer_control'

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
    maintainer='arty',
    maintainer_email='dawkins@usna.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'surfer_interface = surfer_control.surfer_interface:main',
            'surfer_controller = surfer_control.surfer_controller:main'
        ],
    },
)
