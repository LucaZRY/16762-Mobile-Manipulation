import time
import numpy as np
import stretch_body.robot

def safe_sleep(t: float):
    """Simple delay to allow motion to complete when per-joint wait is unavailable."""
    time.sleep(t)

def main():
    robot = stretch_body.robot.Robot()
    robot.startup()

    robot.stow()
    robot.push_command()
    safe_sleep(3.0)

    robot.arm.move_to(0.5)      
    robot.lift.move_to(1.0)     
    robot.push_command()


    robot.arm.wait_until_at_setpoint()
    robot.lift.wait_until_at_setpoint()
    safe_sleep(0.5)

    robot.end_of_arm.move_to('wrist_yaw', np.radians(15))
    robot.push_command()
    safe_sleep(5.0)

    robot.end_of_arm.move_to('wrist_pitch', np.radians(15))
    robot.push_command()
    safe_sleep(5.0)

    robot.end_of_arm.move_to('wrist_roll', np.radians(15))
    robot.push_command()
    safe_sleep(5.0)

    robot.end_of_arm.move_to('stretch_gripper', 20)  
    robot.push_command()
    safe_sleep(5.0)

    robot.end_of_arm.move_to('stretch_gripper', 0)  
    robot.push_command()
    safe_sleep(.0)

    robot.head.move_by('head_pan', np.radians(10))
    robot.push_command()
    safe_sleep(2.0)

    robot.head.move_by('head_tilt', np.radians(20))
    robot.push_command()
    safe_sleep(2.0)

    robot.stow()
    robot.push_command()
    safe_sleep(4.0)

    robot.base.translate_by(0.5)
    robot.push_command()
    safe_sleep(10.0)

    robot.base.rotate_by(np.radians(180))
    robot.push_command()
    safe_sleep(10.0)

    robot.base.translate_by(0.5)
    robot.push_command()
    safe_sleep(10.0)
        
    robot.stow()
    robot.push_command()
    safe_sleep(5.0)

    robot.stop()

if __name__ == "__main__":
    main()
