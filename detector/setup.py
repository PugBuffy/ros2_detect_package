from setuptools import find_packages, setup
from glob import glob

package_name = 'detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [*glob('launch/*.launch.py')])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergey',
    maintainer_email='sergey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = detector.detector:main',
            'camera = detector.camera:main',
            'debug_detector = detector.debug_detector:main'
        ],
    },
)
