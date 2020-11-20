from setuptools import setup

setup(
    name="delta_robot_trampoline",
    version='0.0.2',
    packages=["delta_robot_trampoline"],
    python_requires='>=3',
    download_url='https://github.com/RicoJia/delta_robot_trampoline/archive/0.0.1.tar.gz',
    install_requires=['gym', 'pybullet', 'numpy']
)
