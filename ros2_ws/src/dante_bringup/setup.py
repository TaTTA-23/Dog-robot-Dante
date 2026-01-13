from setuptools import setup

package_name = 'dante_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[],
    entry_points={
        'console_scripts': [
            'dante_node = dante_bringup.dante_node:main'
        ],
    },
)
from setuptools import setup

package_name = 'dante_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@example.com',
    description='Bringup and example nodes for Dog-robot-Dante',
    entry_points={
        'console_scripts': [
            'dante_node = dante_bringup.dante_node:main'
        ],
    },
)
