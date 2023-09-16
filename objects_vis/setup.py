from setuptools import setup
from glob import glob
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['objects_vis'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/objects_vis']),
        ('share/objects_vis',
         ['package.xml', *glob('launch/*.xml'), *glob('config/*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'draw_objects = objects_vis.draw_objects:main',
            'draw_objects3d = objects_vis.draw_objects3d:main',
            'publish_random_objects = objects_vis.publish_random_objects:main',
            'objects2markers = objects_vis.objects2markers:main',
        ],
    },
)

setup(**d)