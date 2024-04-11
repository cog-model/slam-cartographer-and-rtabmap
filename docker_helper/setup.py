from setuptools import setup

setup(
    name='docker_helper',
    version='0.0.1',
    packages=['docker_helper'],
    package_dir={'': 'src'},
    install_requires=[
        'subprocess-tee'
    ],
)