import setuptools

setuptools.setup(
    name="PidController",
    version="0.0.1",
    author="Sebastian Jennen",
    author_email="sj@imagearts.de",
    description="PidController",
    packages=setuptools.find_packages(),
    python_requires='>=3.6',
    install_requires=['requests'],
    scripts=['PidController.py']
)