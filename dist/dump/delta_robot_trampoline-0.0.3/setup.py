from setuptools import setup

setup(
    name="delta_robot_trampoline",
    version='0.0.3',
    packages=["delta_robot_trampoline"],
    package_data={'delta_robot_trampoline': [
    'resources/*']},
    python_requires='>=3',
    install_requires=['gym', 'pybullet', 'numpy']
)
