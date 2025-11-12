import os
import pybullet as p
import pybullet_data as pd
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
    client = p.connect(p.GUI if gui else p.DIRECT)
    p.resetSimulation()
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")

    # Setup default camera for visualization
    p.resetDebugVisualizerCamera(
        cameraDistance=3.5,
        cameraYaw=180,
        cameraPitch=-40,
        cameraTargetPosition=[0, 0.5, 0]
    )
    return client


def load_robot_and_box(robot_path, box_path):
    """Load UR5 robot and colored box into the simulation."""
    arm_id = p.loadURDF(
        robot_path,
        [0, 0, 0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )

    box_id = p.loadURDF(
        box_path,
        [0.5, 0, 0.05],
        p.getQuaternionFromEuler([0, 0, 0]),
        globalScaling=0.1,
        useFixedBase=True
    )

    # ============================================================
    # For ARUCO marker based detection use this section of code
    AR_BOX_URDF = os.path.join(ASSETS_DIR, "ar_marker_box.urdf")
    ARUCO_TEXTURE = os.path.join(ASSETS_DIR, "texture" , "ar_marker_box.png")
    box_id = p.loadURDF(AR_BOX_URDF, basePosition=[0.5, 0, 0.05], baseOrientation=p.getQuaternionFromEuler([0, 0 ,0]), useFixedBase=False)
    texture_id = p.loadTexture(ARUCO_TEXTURE)
    p.changeVisualShape(box_id, -1, textureUniqueId=texture_id)
    pos, orn = p.getBasePositionAndOrientation(box_id)
    new_orn = p.getQuaternionFromEuler([math.radians(90), math.radians(0), math.radians(0)])
    p.resetBasePositionAndOrientation(box_id, pos, new_orn)
    # ============================================================

    ur5_joint_indices = list(range(6))
    init_conf = [0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
    set_joint_positions(arm_id, ur5_joint_indices, init_conf)

    return arm_id, box_id, ur5_joint_indices, init_conf


def setup_camera_parameters():
    """Return default camera parameters (intrinsics and projection)."""
    fov = 60
    image_width, image_height = 224, 224
    aspect = image_width / image_height
    near, far = 0.05, 5
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

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
    print("VISION-BASED CONTROL OF ROBOT ARM")
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
    camera_link_pose = p.getLinkState(arm_id, CAMERA_IDX)[0]
    target_link_pose = p.getLinkState(arm_id, TARGET_IDX)[0]

    # Orientation
    R = Rz(0) @ Ry(0) @ Rx(0)
    up = np.array([0, -1, 0])
    view_matrix = p.computeViewMatrix(camera_link_pose, target_link_pose, R @ up)

    _, _, rgb, depth, _ = p.getCameraImage(
        cam_params["width"], cam_params["height"], view_matrix, cam_params["projection"]
    )
    # ==========================================================================================
    # TODO: Estimate the 3D position (pos) of a box using an external static camera ("eye-in-hand" setup).
    # ==========================================================================================
    pos = None 
    if pos:
        print("Eye-in-Hand detected object:", pos)
    else:
        print("No object detected.")
    # time.sleep(50)
    while True:
        pass
    p.disconnect()

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

    simple_camera_id = p.loadURDF(
        "/home/hoprus/ar523_ws/A3/assets/simple_camera.urdf",
        [cam_x, cam_y, cam_z],
        p.getQuaternionFromEuler([roll, pitch, yaw]),
        useFixedBase=True
    )

    camera_pose = p.getLinkState(simple_camera_id, 0)[0]
    target_pose = p.getLinkState(simple_camera_id, 1)[0]
    up = np.array([0, -1, 0])
    view_matrix = p.computeViewMatrix(camera_pose, target_pose, R @ up)

    _, _, rgb, depth, _ = p.getCameraImage(
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
    while True:
        pass
    p.disconnect()


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
