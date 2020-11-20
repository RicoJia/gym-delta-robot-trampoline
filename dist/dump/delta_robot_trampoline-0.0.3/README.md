# Delta Robot Trampoline

Authors: Rico Ruotong Jia(ruotongjia2020@u.northwestern.edu), Yipeng Pan 

The trampoline bounces a ball using a delta arm trained by DDPG! We also have included a PyBullet + OpenAI Gym environment of a Delta_Arm.  

### Minimum System Requirement
- [PyBullet 3.0.5](https://pybullet.org/wordpress/) (``` sudo pip install pybullet```)
- Python 3.6

### Usage
1. Download this repository:
```$ git clone https://github.com/RicoJia/delta_arm_trampoline.git```
2. Install [pybullet](https://pybullet.org/wordpress/) and [gym](https://gym.openai.com/) 
```$ pip install pybullet && pip install gym```
3. To see the model, do
```$ python3 delta_robot_trampoline.py```
Now, you should be able to see the delta robot starting at home position, 
where each lower leg is touching the ground, and the soccer ball is right on 
top of the platform. 

    ![](media/model_1.png)
4. to see the effect of different joint torques in the model with slider bars, do 
```$ python3 delta_robot_trampoline.py -t```
or 
```$ python3 delta_robot_trampoline.py --test```

