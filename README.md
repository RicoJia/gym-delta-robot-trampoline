# Gym Delta Robot Trampoline

Authors: Rico Ruotong Jia(ruotongjia2020@u.northwestern.edu), Yipeng Pan 

The trampoline bounces a ball using a delta arm trained by DDPG! We also have included a PyBullet + OpenAI Gym environment of a Delta_Arm.  

![](media/model_1.png)

### Minimum System Requirement
- Python 3.6

### Links 
 - Github [https://github.com/RicoJia/gym-delta-robot-trampoline](https://github.com/RicoJia/gym-delta-robot-trampoline)
 - Pypi [https://pypi.org/project/gym-delta-robot-trampoline/](https://pypi.org/project/gym-delta-robot-trampoline/)

### Install
Install with `pip`:

    pip3 install gym_delta_robot_trampoline
    
Or, install from source:

    git clone https://github.com/RicoJia/gym-delta-robot-trampoline
    cd gym-delta-robot-trampoline
    pip install .

### Usage

If you donwload the package from pip, to see the gym environment, in the root of the package on Github [https://github.com/RicoJia/gym-delta-robot-trampoline](https://github.com/RicoJia/gym-delta-robot-trampoline)
Download ```test_main.py``` , then run the script

     python3 test_main.py

You can interact with the environment by clicking and dragging, but note that our simulation is running much 
faster than real time, so you might not see the ball falling to the ground before the episode ends, or 
you might see a higher reward than expected due to small vibrations in our mouse dragging motion.   

#### Training
To train the model, from the root of the package

    cd DDPG 
    python
    
Note that a pre-trained model that is able to bounce the ball 10 times is loaded as a starting point 

#### Demo
To see a demo of the model, from the root of the package, 

    cd DDPG
    python3 main.py -t 
    
This model can bounce the ball 10 times!

<img src="media/ddpg.gif" width="200"/>


