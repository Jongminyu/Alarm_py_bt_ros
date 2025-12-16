import time
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Strict Import: Must build 'rb_interface' package first!
from rb_interface.action import RunOpposite

class RunOppositeActionServer(Node):
    def __init__(self):
        super().__init__('run_opposite_server')
        
        self._action_server = ActionServer(
            self,
            RunOpposite,
            'run_opposite',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Sensor Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
            callback_group=ReentrantCallbackGroup()
        )
        self.latest_scan = None
        
        self.pose_sub = self.create_subscription(
             PoseWithCovarianceStamped,
             '/amcl_pose',
             self.pose_callback,
             10,
             callback_group=ReentrantCallbackGroup()
        )
        self.current_yaw = 0.0

        # Internal State Variables
        self.reset_state()
        
        self.get_logger().info("RunOpposite Action Server Started!")

    def reset_state(self):
        self.state = "IDLE"
        self.start_time = 0
        self.s_curve_timer = 0
        self.s_curve_dir = 1
        self.avoid_state = "IDLE"
        self.avoid_start_time = 0
        self.recover_start_time = 0
        self.recover_direction = 0
        self.pivot_count = 0

    def goal_callback(self, goal_request):
        self.get_logger().info('Received Goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received Cancel request')
        return CancelResponse.ACCEPT

    def scan_callback(self, msg):
        self.latest_scan = msg

    def pose_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.reset_state()
        
        goal = goal_handle.request
        # Params from goal
        target_angle = goal.target_angle
        speed = goal.speed
        # duration = goal.duration (if needed, currently logic uses fixed 15s or 2s)
        
        feedback_msg = RunOpposite.Feedback()
        result = RunOpposite.Result()
        
        twist = Twist()
        
        # Start State
        self.state = "TURN_AWAY"
        self.start_time = time.time()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.message = "Canceled"
                return result
            
            now = time.time()
            
            # --- Logic Copied from RunOpposite.run() ---
            if self.state == "TURN_AWAY":
                if now - self.start_time < 2.0:
                    twist.angular.z = 1.57 # Inverted initial turn
                    twist.linear.x = 0.0
                else:
                    self.state = "WANDER_FLEE"
                    self.start_time = now
                    self.s_curve_timer = now
                    self.avoid_state = "IDLE"
                    self.pivot_count = 0
                    self.get_logger().info("S-Curve Fleeing...")

            elif self.state == "WANDER_FLEE":
                if now - self.start_time > 15.0 and self.avoid_state == "IDLE":
                    self.state = "TURN_BACK"
                    self.start_time = now
                    self.get_logger().info("Turning Back...")
                else:
                    # Wander Logic
                    bias = 0.0
                    
                    # 1. Process Scan
                    min_dist = 99.0
                    critical_obstacle = False
                    obstacle_detected = False
                    left_avg = 10.0
                    right_avg = 10.0
                    
                    if self.latest_scan:
                        num = len(self.latest_scan.ranges)
                        idx_90 = int(num * 0.25)
                        idx_270 = int(num * 0.75)
                        front_ranges = self.latest_scan.ranges[:idx_90] + self.latest_scan.ranges[idx_270:]
                        valid_front = [r for r in front_ranges if r > 0.01 and r < 10.0]
                        
                        if valid_front:
                            min_dist = min(valid_front)
                            if min_dist < 0.18: critical_obstacle = True
                            elif min_dist < 0.80: obstacle_detected = True
                            
                        # Sector averages
                        def get_avg(start, end):
                            s_idx = int((start % 360)/360 * num)
                            e_idx = int((end % 360)/360 * num)
                            if s_idx < e_idx: sect = self.latest_scan.ranges[s_idx:e_idx]
                            else: sect = self.latest_scan.ranges[s_idx:] + self.latest_scan.ranges[:e_idx]
                            v = [r for r in sect if r > 0.01 and r < 10.0]
                            return sum(v)/len(v) if v else 10.0
                            
                        left_avg = get_avg(45, 135)
                        right_avg = get_avg(225, 315)

                    # 2. Avoidance State Machine
                    if self.avoid_state == "PIVOTING":
                        if now - self.recover_start_time < 1.0:
                            twist.linear.x = 0.0
                            twist.angular.z = 2.0 * self.recover_direction
                        else:
                            self.avoid_state = "WAIT_FOR_SCAN"
                            self.recover_start_time = now
                            
                    elif self.avoid_state == "ESCAPE_BACKUP":
                        if now - self.recover_start_time < 0.8:
                            twist.linear.x = -0.15
                            twist.angular.z = 0.0
                        else:
                            self.avoid_state = "WAIT_FOR_SCAN"
                            self.recover_start_time = now
                            
                    elif self.avoid_state == "WAIT_FOR_SCAN":
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        if now - self.recover_start_time > 0.3:
                            self.avoid_state = "IDLE"
                            
                    elif self.avoid_state == "IDLE":
                        if critical_obstacle:
                            if self.pivot_count >= 3:
                                self.avoid_state = "ESCAPE_BACKUP"
                                self.recover_start_time = now
                                self.pivot_count = 0
                            else:
                                self.avoid_state = "PIVOTING"
                                self.recover_start_time = now
                                self.pivot_count += 1
                                self.recover_direction = 1.0 if left_avg < right_avg else -1.0
                        else:
                            # S-Curve
                            self.pivot_count = 0
                            if obstacle_detected:
                                bias = 1.8 if left_avg < right_avg else -1.8
                                twist.linear.x = speed * 0.5
                            else:
                                twist.linear.x = speed
                                
                            if now - self.s_curve_timer > 0.5:
                                self.s_curve_dir *= -1
                                self.s_curve_timer = now
                                
                            twist.angular.z = bias + (1.0 * self.s_curve_dir)
                            twist.angular.z = max(min(twist.angular.z, 2.0), -2.0)

            elif self.state == "TURN_BACK":
                if now - self.start_time < 2.0:
                    twist.angular.z = -1.57 # Inverted return turn
                    twist.linear.x = 0.0
                else:
                    self.state = "IDLE"
                    twist.angular.z = 0.0
                    twist.linear.x = 0.0
                    self.pub.publish(twist)
                    
                    goal_handle.succeed()
                    result.message = "Success"
                    return result

            # Publish Control
            self.pub.publish(twist)
            
            # Publish Feedback
            feedback_msg.current_state = self.state
            feedback_msg.time_elapsed = now - self.start_time
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.01) # Loop rate (100Hz) for smoother control

def main(args=None):
    rclpy.init(args=args)
    server = RunOppositeActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)

if __name__ == '__main__':
    main()
