from setuptools import setup

package_name = 'control_plotting'
submodules = "control_plotting/trajectory"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xma42',
    maintainer_email='mxcheng@umich.edu',
    description='Control Reproduce plotting',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_plotting = control_plotting.control_plotting:main',
        ],
    },
)
