import os
RLPX4_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
RLPX4_ENVS_DIR = os.path.join(RLPX4_ROOT_DIR, 'traj_tools')

print("rlPx4Controller_ROOT_DIR", RLPX4_ENVS_DIR)