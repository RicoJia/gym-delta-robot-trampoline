from setuptools import setup

setup(
    name="delta_robot_trampoline",
    version='0.0.1',
    packages=["delta_robot_trampoline"],
    python_requires='>=3',
    install_requires=['gym', 'pybullet', 'numpy']
)
