from gym_delta_robot_trampoline.resources.delta_robot_trampoline import test_robot
#from gym_delta_robot_trampoline.envs.gym_delta_robot_trampoline_env import DeltaRobotTrampolineEnv

from gym.envs.registration import register
register(
    id='delta_robot_trampoline-v0',
    entry_point='gym_delta_robot_trampoline.envs:DeltaRobotTrampolineEnv',
)
