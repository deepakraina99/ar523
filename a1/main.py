
import pybullet as p
import pybullet_data
import time
import numpy as np
import random
from utils import *
import argparse
import math
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = os.path.join(current_dir, "assest")
ur5_joint_indices = list(range(0,6))  

def execute_path_with_trail(body_id, link_index, joints, path, step_sleep, color_rgb, linewidth=2.0, lifetime=0):
    ####################################################
    # TODO your code to visulalize the path with a trail
    ####################################################
    pass

def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--rrt', action='store_true', default=False)
    parser.add_argument('--rrt_task', action='store_true', default=False)
    parser.add_argument('--birrt_task', action='store_true', default=False)
    args = parser.parse_args()
    return args

def rrt():
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    pass

def rrt_task_space():
    ##############################################################
    # TODO your code to implement the rrt in task space algorithm
    ##############################################################
    pass

def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
    pass

def birrt_task_space():
    ###############################################################
    # TODO your code to implement the birrt in task space algorithm
    ###############################################################
    pass

# -------------------------------
#  IK helpers
# -------------------------------
def ik_conf(target_pos, target_orn):
    q = p.calculateInverseKinematics(ur5_robo, 7 , target_pos, target_orn)
    return q  # only arm joints

# -------------------------------
#  Pipeline
# -------------------------------

if __name__ == "__main__":
    args = get_args()
    # -------------------------------
    # 1. Start PyBullet
    # -------------------------------
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # -------------------------------
    # 2. Load environment
    # -------------------------------
    plane = p.loadURDF("plane.urdf")
    flags = p.URDF_USE_INERTIA_FROM_FILE

    ur5_robo = p.loadURDF(os.path.join(assets_dir, "ur5.urdf"),
        basePosition=[0, 0, 0.38],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        flags=flags,
        useFixedBase=True
    )

    table_position = [0.45, 0.0, 0.0]
    table_id = p.loadURDF(os.path.join(assets_dir, "table", "table.urdf"),
                        table_position, p.getQuaternionFromEuler([0, 0, 1.5708]),
                        flags=flags, useFixedBase=True)

    cube_pos= [[0.65, 0, 0.35], [0.4, -0.2, 0.35], [0.4, 0.2, 0.35]]
    cube_id_1 = p.loadURDF(os.path.join(assets_dir, "cube_and_square", "cube_small_yellow.urdf"),
                        cube_pos[0], p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
    cube_id_2 = p.loadURDF(os.path.join(assets_dir, "cube_and_square", "cube_small_yellow.urdf"),
                        cube_pos[1], p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
    cube_id_3 = p.loadURDF(os.path.join(assets_dir, "cube_and_square", "cube_small_yellow.urdf"),
                    cube_pos[2], p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

    # -------------------------------
    # 3. Collision function
    # -------------------------------
    obstacles = [ plane, table_id,cube_id_1,cube_id_2,cube_id_3]
    collision_fn = get_collision_fn(ur5_robo, ur5_joint_indices, obstacles=obstacles,
                                    attachments=[], self_collisions=False,
                                    disabled_collisions=set())

    # -------------------------------
    # 4. Defining three targets
    # -------------------------------
    # Inital robot poisition
    init_conf = [0,-math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
    set_joint_positions(ur5_robo, ur5_joint_indices, init_conf)

    # Target position 1
    start = [0.3,-0.375,0.45]
    quat = p.getQuaternionFromEuler([0,0,0])
    start_conf = ik_conf(start, quat)
    start_marker = draw_sphere_marker(position=start, radius=0.02, color=[0, 0, 1, 0.5])

    # Target position 2
    intermediate = [0.45, 0.0, 0.45]
    intermediate_conf = ik_conf(intermediate, quat)
    goal_marker = draw_sphere_marker(position=intermediate, radius=0.02, color=[0, 1, 0, 0.5])

    # Target position 3
    end = [0.6,0.375,0.55]
    end_conf = ik_conf(end, quat)
    goal_marker = draw_sphere_marker(position=end, radius=0.02, color=[1, 0, 0, 0.5])

    # Checking collision with tinitial environment state
    print("Initial collision?", collision_fn(init_conf))
    print("Start collision?", collision_fn(start_conf))
    print("Intermediate collision?", collision_fn(intermediate_conf))
    print("Place collision?", collision_fn(end_conf))

    # -------------------------------
    # 5. Path planning to reach 3 targets
    # -------------------------------
    if args.birrt:
        path1, path2, path3 = birrt()
    elif args.birrt_task:
        path1, path2, path3 = birrt_task_space()
    elif args.rrt_task:
        path1, path2, path3 = rrt_task_space()
    else:
        path1, path2, path3 = rrt()

    timeStep = 0.5

    # -------------------------------
    # 6. Move robot to track paths
    # -------------------------------
    while True:
        # Blue: home -> pick
        execute_path_with_trail(ur5_robo, 7, ur5_joint_indices, path1, timeStep, color_rgb=[0, 0, 1])
        # Green: pick -> mid
        execute_path_with_trail(ur5_robo, 7, ur5_joint_indices, path2, timeStep, color_rgb=[0, 1, 0])
        # Red: mid -> place
        execute_path_with_trail(ur5_robo, 7, ur5_joint_indices, path3, timeStep, color_rgb=[1, 0, 0])
        print("finished!")
        time.sleep(6)
