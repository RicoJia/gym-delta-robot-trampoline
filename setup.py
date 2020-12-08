from setuptools import setup, find_packages
from pathlib import Path
setup(
    name="gym_delta_robot_trampoline",
    author="Rico Ruotong Jia",
    author_email="ruotongjia2020@u.northwestern.edu",
    version='0.1.7',
    description="An OpenAI Gym Env for Delta Robot",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    url="https://github.com/RicoJia/gym-delta-robot-trampoline",
    packages=find_packages(include="gym_delta_robot_trampoline*"),
    # packages=["gym_delta_robot_trampoline"],
    # package_data={'gym_delta_robot_trampoline': [
    # 'resources/*']},
    install_requires=['gym', 'pybullet', 'numpy', "matplotlib"],
    classifiers=[
    "Programming Language :: Python :: 3",
    "License :: OSI Approved :: BSD License",
    "Operating System :: OS Independent",
    ],
    python_requires='>=3.6'
)

