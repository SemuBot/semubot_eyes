from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semubot_eyes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the images folder to the share directory of your package
        (os.path.join('share', package_name, 'images'), glob('images/*.*')),

        # Install test_speech.wav to the share directory of your package
        (os.path.join('share', package_name), ['test_speech.wav']),

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timur',
    maintainer_email='nizamovofficial@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eye_controller = semubot_eyes.eye_controller:main'
        ],
    },
)
