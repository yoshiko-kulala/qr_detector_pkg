from setuptools import setup

package_name = 'qr_detector_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qr_detector.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='WRS Team',
    maintainer_email='example@example.com',
    description='QR code detection package for ROS 2 Humble.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector = qr_detector_pkg.node_qr_detector:main',
        ],
    },
)
