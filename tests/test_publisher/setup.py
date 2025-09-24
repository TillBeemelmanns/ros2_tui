from setuptools import setup

package_name = 'ros2_tui_test'

setup(
    name=package_name,
    version='0.1.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dummy_publishers.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Till Beemelmanns',
    maintainer_email='till.beemelmanns@example.com',
    description='Test publishers and parameter setters for debugging ros2_tui',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_publisher = ros2_tui_test.dummy_publisher:main',
            'param_setter = ros2_tui_test.param_setter:main',
            'multi_publisher = ros2_tui_test.multi_publisher:main',
        ],
    },
)