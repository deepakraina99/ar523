import os
import pybullet
import pybullet_data
import cv2
import numpy as np
import time
import math
from utils import *

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(CURRENT_DIR, "assets")

# ============================================================
# --------------- BASIC UTILITY AND SETUP --------------------
# ============================================================

def init_simulation(gui=True):
    """Initialize the PyBullet simulation with a plane and default camera view."""
    client = pybullet.connect(pybullet.GUI if gui else pybullet.DIRECT)
    pybullet.resetSimulation()
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.8)
    pybullet.setTimeStep(1.0 / 240.0)
    pybullet.loadURDF("plane.urdf")

    # Setup default camera for visualization
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=3.5,
        cameraYaw=180,
        cameraPitch=-40,
        cameraTargetPosition=[0, 0.5, 0]
    )
    return client


def load_robot_and_box(robot_path, box_path):
    """Load UR5 robot and colored box into the simulation."""
    arm_id = pybullet.loadURDF(
        robot_path,
        [0, 0, 0],
        pybullet.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )

    color_box_id = pybullet.loadURDF(
        box_path,
        [0.5, 0, 0.05],
        pybullet.getQuaternionFromEuler([0, 0, 0]),
        globalScaling=0.1,
        useFixedBase=True
    )

    ur5_joint_indices = list(range(6))
    init_conf = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
    set_joint_positions(arm_id, ur5_joint_indices, init_conf)

    return arm_id, color_box_id, ur5_joint_indices, init_conf


def setup_camera_parameters():
    """Return default camera parameters (intrinsics and projection)."""
    fov = 60
    image_width, image_height = 224, 224
    aspect = image_width / image_height
    near, far = 0.05, 5
    projection_matrix = pybullet.computeProjectionMatrixFOV(fov, aspect, near, far)

    # focal length in pixels
    fov_rad = np.deg2rad(fov)
    f = (image_height / 2) / np.tan(fov_rad / 2)

    return {
        "fov": fov,
        "width": image_width,
        "height": image_height,
        "aspect": aspect,
        "near": near,
        "far": far,
        "f": f,
        "projection": projection_matrix
    }

# ============================================================
# --------------- CONFIGURATION SELECTION --------------------
# ============================================================

def get_user_choice():
    print("\n" + "="*60)
    print("ROBOT ARM OBJECT POSE ESTIMATION")
    print("="*60)
    print("\nSelect camera configuration:")
    print("1. Eye-in-Hand (camera mounted on robot arm)")
    print("2. Eye-to-Hand (fixed camera observing workspace)")
    print("="*60)

    while True:
        choice = input("\nEnter your choice (1 or 2): ").strip()
        if choice == '1': return 'eye_in_hand'
        if choice == '2': return 'eye_to_hand'
        print("Invalid input. Please enter 1 or 2.")


# ============================================================
# --------------- EYE-IN-HAND ESTIMATION ---------------------
# ============================================================

def run_eye_in_hand():
    arm_id, color_box_id, ur5_joints, init_conf = load_robot_and_box(
        os.path.join(ASSETS_DIR, "ur5.urdf"), os.path.join(ASSETS_DIR, "simple_box.urdf")
    )
    cam_params = setup_camera_parameters()
    print("\nRunning Eye-in-Hand configuration...")
    CAMERA_IDX, TARGET_IDX = 6, 7
    camera_link_pose = pybullet.getLinkState(arm_id, CAMERA_IDX)[0]
    target_link_pose = pybullet.getLinkState(arm_id, TARGET_IDX)[0]

    # Orientation
    R = Rz(0) @ Ry(0) @ Rx(0)
    up = np.array([0, -1, 0])
    view_matrix = pybullet.computeViewMatrix(camera_link_pose, target_link_pose, R @ up)

    _, _, rgb, depth, _ = pybullet.getCameraImage(
        cam_params["width"], cam_params["height"], view_matrix, cam_params["projection"]
    )
    # ==========================================================================================
    # TODO: Estimate the 3D position (pos) of a box using a camera mounted on arm ("eye-in-hand" setup).
    # ==========================================================================================
    pos = None 
    if pos:
        print("Eye-in-Hand detected object:", pos)
    else:
        print("No object detected.")
    time.sleep(50)
    pybullet.disconnect()


# ============================================================
# --------------- EYE-TO-HAND ESTIMATION ---------------------
# ============================================================

def run_eye_to_hand():
    arm_id, color_box_id, ur5_joints, init_conf = load_robot_and_box(
        os.path.join(ASSETS_DIR, "ur5.urdf"), os.path.join(ASSETS_DIR, "simple_box.urdf")
    )
    cam_params = setup_camera_parameters()
    print("\nRunning Eye-to-Hand configuration...")
    cam_x, cam_y, cam_z = 0.0, 0.0, 1.5
    roll, pitch, yaw = math.radians(180), 0.0, 0.0
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    simple_camera_id = pybullet.loadURDF(
        "/home/hoprus/ar523_ws/A3/assets/simple_camera.urdf",
        [cam_x, cam_y, cam_z],
        pybullet.getQuaternionFromEuler([roll, pitch, yaw]),
        useFixedBase=True
    )

    camera_pose = pybullet.getLinkState(simple_camera_id, 0)[0]
    target_pose = pybullet.getLinkState(simple_camera_id, 1)[0]
    up = np.array([0, -1, 0])
    view_matrix = pybullet.computeViewMatrix(camera_pose, target_pose, R @ up)

    _, _, rgb, depth, _ = pybullet.getCameraImage(
        cam_params["width"], cam_params["height"], view_matrix, cam_params["projection"]
    )

    # ==========================================================================================
    # TODO: Estimate the 3D position (pos) of a box using an external static camera ("eye-to-hand" setup).
    # ==========================================================================================
    pos = None 
    if pos:
        print("Eye-to-Hand detected object:", pos)
    else:
        print("No object detected.")
    time.sleep(50)
    pybullet.disconnect()


# ============================================================
# ----------------------- MAIN -------------------------------
# ============================================================

def main():
    client = init_simulation()
    choice = get_user_choice()
    if choice == 'eye_in_hand':
        run_eye_in_hand()
    else:
        run_eye_to_hand()


if __name__ == "__main__":
    main()
