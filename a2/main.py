import pybullet as p
import pybullet_data
import numpy as np
import os
from utils import (reset_joint_states, enable_force_torque_sensor, force_torque_sensing)

def load_robot(urdf_dir):
    """
    Load UR5 robot with end-effector and reset joint states.
    """
    robot = p.loadURDF(os.path.join(urdf_dir, 'ur', 'ur5-ee.urdf'),
                       useFixedBase=True, basePosition=[0, 0, 0.5],
                       flags=p.URDF_USE_SELF_COLLISION)
    control_joint_indices = [0, 1, 2, 3, 4, 5]
    tool_index = 6
    reset_joint_states(robot, control_joint_indices, [0.0, -1.57, 1.57, -1.57, -1.57, -1.57])
    for _ in range(100):
        p.stepSimulation()
    return robot, control_joint_indices, tool_index

def load_box(position, orientation=(0,0,0,1), mass=1., dimensions=(1.,1.,1.), color=(0.5,0.5,0.5,1)):
    """
    Utility to load a box with collision and visual shape.
    """
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=np.array(dimensions)/2.)
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=np.array(dimensions)/2., rgbaColor=color)
    box = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_shape,
                            baseVisualShapeIndex=visual_shape, basePosition=position,
                            baseOrientation=p.getQuaternionFromEuler((0.01,0,0)))
    return box

def load_mesh(filename, position, orientation=(0, 0, 0, 1), mass=1., scale=(1., 1., 1.),
              color=None, with_collision=True, flags=None, *args, **kwargs):
    kwargs = {}
    if flags is not None:
        kwargs['flags'] = flags
    # create collision shape if specified
    collision_shape = None
    if with_collision:
        collision_shape = p.createCollisionShape(p.GEOM_MESH, fileName=filename, meshScale=scale,
                                                        **kwargs)
    if color is not None:
        kwargs['rgbaColor'] = color
    # create visual shape
    visual_shape = p.createVisualShape(p.GEOM_MESH, fileName=filename, meshScale=scale, **kwargs)
    # create body
    if with_collision:
        mesh = p.createMultiBody(baseMass=mass,
                                        baseCollisionShapeIndex=collision_shape,
                                        baseVisualShapeIndex=visual_shape,
                                        basePosition=position,
                                        baseOrientation=orientation)
    else:
        mesh = p.createMultiBody(baseMass=mass,
                                        baseVisualShapeIndex=visual_shape,
                                        basePosition=position,
                                        baseOrientation=orientation)
    return mesh
    
def create_world(asset_dir):
    """
    Setup the simulation environment with plane, table, and a visual marker.
    """
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setRealTimeSimulation(0)

    p.loadURDF(os.path.join(asset_dir, 'plane', 'plane.urdf'))
    # load_box(position=np.array([0.6, 0., 0.25]), dimensions=(0.7, 1, 0.5), mass=0)
    load_mesh(filename=os.path.join(asset_dir, 'box_slanted.obj'), position=np.array([0.3, 0, 0.25]), orientation=p.getQuaternionFromEuler([0, 0, np.pi/2]), mass=0)

if __name__ == "__main__":
    asset_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'assets')
    p.connect(p.GUI)
    time_step = 1./240
    p.setTimeStep(time_step)

    # Create environment and robot
    create_world(asset_dir)
    robot, controllable_joints, tool = load_robot(asset_dir)
    enable_force_torque_sensor(robot, tool)

    # Simulation variables
    iteration = 0
    interaction_type = 'dynamic'  # TODO: change to 'static' to test static interaction
    fixed_target_position = np.array([0.5, 0.1, 0.75])
    dynamic_target_position = fixed_target_position.copy()
    fixed_orientation = np.array([1,0,0,0])
    Fz_desired = 20

    # Circular motion parameters
    r = 0.1
    w = 0.0001

    print("Starting simulation. Press Ctrl+C to stop.")

    while True:
        # Measure current force
        F = force_torque_sensing(robot, tool)
        Fz_current = F[2]


        # Get end-effector state (for reference / optional use)
        x = np.asarray(p.getLinkState(robot, tool)[0])
        dx = np.asarray(p.getLinkState(robot, tool, computeLinkVelocity=1)[6])
        o = np.asarray(p.getLinkState(robot, tool)[1])
        do = np.asarray(p.getLinkState(robot, tool, computeLinkVelocity=1)[7])

        # -------------------------------------------------
        # TODO: Implement Admittance or admittance Control
        # -------------------------------------------------

        # select interaction type here
        if interaction_type == 'static':
            dynamic_target_position[:2] = np.array([fixed_target_position[0], fixed_target_position[1]])

        elif interaction_type == "dynamic":
            dynamic_target_position[:2] = np.array([
                fixed_target_position[0] - r * np.sin(w * iteration + np.pi/2),
                fixed_target_position[1] + r * np.cos(w * iteration + np.pi/2),
            ])

        # Apply joint position control
        # p.setJointMotorControlArray(robot, controllable_joints,
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=joint_positions)
        # OR
        # Apply joint torques control
        # p.setJointMotorControlArray(robot, controllable_joints,
        #                             controlMode=p.TORQUE_CONTROL,
        #                             forces=joint_torques)

        p.stepSimulation()
        iteration += 1

        if iteration % 100 == 0:
            print(f"Step {iteration}, Fz_measured={Fz_current:.2f}")
