import time
import numpy as np
import rclpy
import hello_helpers.hello_misc as hm

def wait_for_joint_state(node, timeout_s=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if getattr(node, "joint_state", None) is not None and len(node.joint_state.name) > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False

rclpy.init()
node = hm.HelloNode.quick_create('lab1_ros2')

# task begin
try:
    node.stow_the_robot()
    time.sleep(2.0)

    # Extend arm + lift together
    node.move_to_pose({'joint_arm': 0.5,
                       'joint_lift': 1.1}, blocking=True)

    # Wrist motors one at a time
    node.move_to_pose({'joint_wrist_yaw': np.radians(30)}, blocking=True)
    node.move_to_pose({'joint_wrist_pitch': np.radians(30)}, blocking=True)
    node.move_to_pose({'joint_wrist_roll': np.radians(30)}, blocking=True)

    # Open / close gripper
    node.move_to_pose({'joint_gripper_finger_left': 0.6}, blocking=True)
    node.move_to_pose({'joint_gripper_finger_right': -0.6}, blocking=True)

    # Head pan/tilt relative motions (read current, add delta)
    if not wait_for_joint_state(node, timeout_s=5.0):
        raise RuntimeError("joint_state not available; cannot compute relative head pan/tilt.")

    pan_i = node.joint_state.name.index('joint_head_pan')
    tilt_i = node.joint_state.name.index('joint_head_tilt')

    current_pan = node.joint_state.position[pan_i]
    node.move_to_pose({'joint_head_pan': current_pan + np.radians(45)}, blocking=True)

    current_tilt = node.joint_state.position[tilt_i]
    node.move_to_pose({'joint_head_tilt': current_tilt + np.radians(45)}, blocking=True)

    # Back to stow
    node.stow_the_robot()
    time.sleep(2.0)

    # Base moves (pose keys)
    node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)
    node.move_to_pose({'rotate_mobile_base': np.radians(180)}, blocking=True)
    node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

    print("task completed")

finally:
    node.destroy_node()
    rclpy.shutdown()
