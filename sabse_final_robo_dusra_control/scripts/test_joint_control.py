#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import time

class JointControllerTest(Node):
    def __init__(self):
        super().__init__('joint_controller_test')
        
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info("Waiting for arm controller action server...")
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm controller action server not available")
            sys.exit(1)
        
        self.get_logger().info("Arm controller connected!")
        self.goal_completed = True

    def move_arm(self, positions, duration):
        goal = FollowJointTrajectory.Goal()
        
        goal.trajectory.joint_names = [
            'base_rotation_joint',
            'shoulder_joint',
            'wrist_joint',
            'forearm_joint',
            'elbow_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        
        goal.trajectory.points.append(point)
        
        goal.goal_tolerance = []
        for i in range(len(goal.trajectory.joint_names)):
            tolerance = JointTolerance()
            tolerance.name = goal.trajectory.joint_names[i]
            tolerance.position = 0.3
            tolerance.velocity = 0.3
            goal.goal_tolerance.append(tolerance)
        
        self.get_logger().info(f"Sending goal: {positions}")
        send_goal_future = self.arm_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
            
        self.get_logger().info("Goal accepted!")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().error(f"Goal failed with status: {status}")
        self.goal_completed = True

def main():
    rclpy.init()
    controller = JointControllerTest()
    time.sleep(2.0)
    
    # Only make tiny movements, 0.05 radians at a time
    try:
        # Sequence: Start at all zeros
        # [base_rotation, shoulder, wrist, forearm, elbow]
        controller.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], 5)
        time.sleep(6)
        
        # Tiny base rotation of 0.05 radians (about 3 degrees)
        controller.move_arm([0.0, 0.2, 0.0, 0.0, 0.0], 15)
        time.sleep(16)
        
        controller.move_arm([0.5, 0.0, 0.0, 0.0, 0.0], 15)
        time.sleep(16)
        

        controller.move_arm([0.0, 0.0, 0.5, 0.0, 0.0], 15)
        time.sleep(16)
        
        # Back to home
        controller.move_arm([0.0, 0.0, 0.0, 0.0, 0.0], 15)
        time.sleep(16)
        
        # Spin for a while to process any remaining callbacks
        timeout = time.time() + 5.0
        while time.time() < timeout:
            rclpy.spin_once(controller, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()