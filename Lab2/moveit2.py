import rclpy, time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from hello_helpers.hello_misc import HelloNode
import moveit2_utils

# Make sure to run `ros2 launch stretch_core stretch_driver.launch.py`

class MoveMe(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.main('move_me', 'move_me', wait_for_first_pointcloud=False)
        self.stow_the_robot()

        planning_group = 'mobile_base_arm'
        moveit, moveit_plan, planning_params = moveit2_utils.setup_moveit(planning_group)
        
        # Define the trajectory steps
        # Pose 0 is the current stowed position
        # We'll execute 4 planning steps to reach poses 1, 2, 3, and 4
        
        for i in range(4):
            print(f'--- Planning Step {i} ---')
            goal_state = RobotState(moveit.get_robot_model())

            # Ordering: [x, y, theta, lift, arm/4, arm/4, arm/4, arm/4, yaw, pitch, roll]
            # For driving the base: the positive x-axis is pointing out of the front of the robot (the flat side of the base). 
            # Positive y-axis is on the left of the robot (opposite direction the arm is facing).
            
            if i == 0:
                # Pose 0 → 1: Lift arm to 0.5 m
                goal_state.set_joint_group_positions(planning_group, 
                    [0.0, 0.0, 0.0,  # Base doesn't move
                    0.5,  # Lift to 0.5 m
                    self.get_joint_pos('joint_arm_l3'), 
                    self.get_joint_pos('joint_arm_l2'), 
                    self.get_joint_pos('joint_arm_l1'), 
                    self.get_joint_pos('joint_arm_l0'), 
                    self.get_joint_pos('joint_wrist_yaw'), 
                    self.get_joint_pos('joint_wrist_pitch'), 
                    self.get_joint_pos('joint_wrist_roll')]
                )
                
            elif i == 1:
                # Pose 1 → 2: Extend arm to 0.4 m (0.1 m per segment)
                goal_state.set_joint_group_positions(planning_group, 
                    [0.0, 0.0, 0.0,  # Base doesn't move
                    self.get_joint_pos('joint_lift'),  # Keep current lift position
                    0.1, 0.1, 0.1, 0.1,  # Extend each arm segment to 0.1 m (total 0.4 m)
                    self.get_joint_pos('joint_wrist_yaw'), 
                    self.get_joint_pos('joint_wrist_pitch'), 
                    self.get_joint_pos('joint_wrist_roll')]
                )
                
            elif i == 2:
                # Pose 2 → 3: Rotate wrist 45 degrees on each of 3 axes
                goal_state.set_joint_group_positions(planning_group, 
                    [0.0, 0.0, 0.0,  # Base doesn't move
                    self.get_joint_pos('joint_lift'),  # Keep current lift
                    self.get_joint_pos('joint_arm_l3'), 
                    self.get_joint_pos('joint_arm_l2'), 
                    self.get_joint_pos('joint_arm_l1'), 
                    self.get_joint_pos('joint_arm_l0'),  # Keep current arm extension
                    self.get_joint_pos('joint_wrist_yaw') + np.radians(45),  # Add 45 degrees to current yaw
                    self.get_joint_pos('joint_wrist_pitch') + np.radians(45),  # Add 45 degrees to current pitch
                    self.get_joint_pos('joint_wrist_roll') + np.radians(45)]  # Add 45 degrees to current roll
                )
                
            elif i == 3:
                # Pose 3 → 4: Return all arm motors to stow pose
                goal_state.set_joint_group_positions(planning_group, 
                    [0.0, 0.0, 0.0,  # Base doesn't move
                    0.0,  # Lower lift to stow
                    0.0, 0.0, 0.0, 0.0,  # Retract all arm segments
                    0.0,  # Wrist yaw to neutral
                    0.0,  # Wrist pitch to neutral
                    0.0]  # Wrist roll to neutral
                )

            moveit_plan.set_start_state_to_current_state()
            moveit_plan.set_goal_state(robot_state=goal_state)
            
            plan = moveit_plan.plan(parameters=planning_params)
            # print(plan.trajectory.get_robot_trajectory_msg())
    
            self.execute_plan(plan)

    def execute_plan(self, plan):
        # NOTE: You don't need to edit this function
        processor = moveit2_utils.TrajectoryProcessor()
        segments = processor.process_trajectory(plan, self.joint_state)

        for i, goal_traj in enumerate(segments):
            # print(goal_traj)
            # time.sleep(2.0)
            self.get_logger().info(f"Executing segment {i+1}/{len(segments)} (Mode: {self._detect_mode(goal_traj)})")
            
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = goal_traj
            
            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Segment {i+1} rejected!")
                break
            
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            res = res_future.result()
            
            if res.result.error_code != res.result.SUCCESSFUL:
                self.get_logger().error(f"Segment {i+1} failed! Code: {res.result.error_code}")
                break

    def get_joint_pos(self, joint_name):
        return self.joint_state.position[self.joint_state.name.index(joint_name)]
        
    def _detect_mode(self, traj):
        if 'translate_mobile_base' in traj.joint_names: return 'TRANSLATE'
        if 'rotate_mobile_base' in traj.joint_names: return 'ROTATE'
        return 'ARM_ONLY'

if __name__ == '__main__':
    MoveMe()