from modules.base_bt_nodes import Status, Node
from modules.base_bt_nodes_ros import ActionWithROSAction

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import math
import time
import json
import numpy as np

try:
    from cv_bridge import CvBridge
except ImportError:
    print("❌ ERROR: cv_bridge not found!")
    CvBridge = None

# Strict Import: Must build 'rb_interface' package first!
from rb_interface.action import RunOpposite as RunOppositeAction

class RunOpposite(ActionWithROSAction):
    def __init__(self, node_type, agent, name=None, target_angle="{user_angle}", speed=1.0):
        actual_name = name if name else node_type
        # Pass (ActionType, ActionName)
        super().__init__(actual_name, agent, (RunOppositeAction, 'run_opposite'))
        
        self.target_angle_key = target_angle
        self.speed = float(speed)

    def _build_goal(self, agent, blackboard):
        print(f"[{self.name}] Building Action Goal...")
        goal = RunOppositeAction.Goal()
        
        # Resolve target angle from blackboard if it's a key
        t_angle = 0.0
        if isinstance(self.target_angle_key, str) and self.target_angle_key.startswith("{"):
            key = self.target_angle_key.strip("{}")
            t_angle = float(blackboard.get(key, 0.0))
        else:
            t_angle = float(self.target_angle_key)
            
        goal.target_angle = t_angle
        goal.speed = self.speed
        goal.duration = 15.0 # Fixed duration or make it a param
        print(f"[{self.name}] Sending Goal: Angle={t_angle}, Speed={self.speed}")
        return goal

    def _on_running(self, agent, blackboard):
        # Optional: Print feedback
        pass

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_ABORTED:
            print(f"[{self.name}] RunOpposite ABORTED by Server.")
            return Status.FAILURE
        elif status_code == GoalStatus.STATUS_CANCELED:
            print(f"[{self.name}] RunOpposite CANCELED.")
            return Status.SUCCESS # Cancelled successfully
        
        return Status.SUCCESS

    def halt(self):
        print(f"[{self.name}] 🚨 RunOpposite Client: HALT called! Sending Cancel...")
        super().halt()

# =========================================================
# Preserved Classes (Rotate, Wander, DepthAvoidance)
# =========================================================

class Rotate(Node):
    def __init__(self, node_type, agent, name=None, angle=3.14, speed=1.5):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.target_angle = float(angle)
        self.speed = float(speed)
        self.pub = self.ros.node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pose_sub = self.ros.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.current_yaw = None
        self.start_yaw = None
        self.is_running = False

    def pose_callback(self, msg):
        q = msg.pose.pose.orientation
        # Yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    async def run(self, agent, blackboard):
        if not self.is_running:
            self.is_running = True
            self.start_yaw = self.current_yaw
            self.start_time = time.time() # Start timer
            print(f"[{self.name}] Starting rotation ({self.target_angle:.2f} rad)...")
        
        twist = Twist()
        elapsed = time.time() - self.start_time
        
        # Timeout Safety (3.0s)
        if elapsed > 2.1:
             print(f"[{self.name}] Rotation Timeout. Forcing Success.")
             twist.angular.z = 0.0
             self.pub.publish(twist)
             self.is_running = False
             # blackboard['stop_mode'] = True  <-- REMOVED: Don't stop here.
             return Status.SUCCESS

        if self.current_yaw is not None and self.start_yaw is not None:
            # Calculate target absolute yaw
            tgt = self.start_yaw + self.target_angle
            # Normalize tgt
            while tgt > math.pi: tgt -= 2*math.pi
            while tgt < -math.pi: tgt += 2*math.pi
            
            # Error
            err = tgt - self.current_yaw
            while err > math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi
            
            if abs(err) < 0.1:
                twist.angular.z = 0.0
                self.is_running = False
                self.pub.publish(twist)
                print(f"[{self.name}] Rotation complete.")
                # blackboard['stop_mode'] = True  <-- REMOVED
                return Status.SUCCESS
            
            twist.angular.z = 1.5 * err
            twist.angular.z = max(min(twist.angular.z, self.speed), -self.speed)
            if abs(twist.angular.z) < 0.3:
                twist.angular.z = 0.3 if err > 0 else -0.3
        else:
            # Fallback time based (Blind Turn)
            twist.angular.z = self.speed
            # 3.14 rad / 1.5 rad/s ~= 2.1s. 
            # We rely on the 3.0s timeout above to stop it.
            pass

        self.pub.publish(twist)
        return Status.RUNNING

class Wander(Node):
    def __init__(self, node_type, agent, name=None, speed=0.2):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.speed = float(speed)
        self.pub = self.ros.node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to LiDAR
        self.scan_sub = self.ros.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        )
        self.latest_scan = None
        
        self.last_change_time = 0
        self.current_angular = 0.0
        self.s_curve_timer = 0
        self.s_curve_dir = 1
        
        # Avoidance State Machine
        self.avoid_state = "IDLE" # IDLE, BACKING, SCANNING, TURNING, FORWARD_SHORT
        self.avoid_start_time = 0
        self.avoid_direction = 0.0

    def scan_callback(self, msg):
        self.latest_scan = msg

    async def run(self, agent, blackboard):
        # Check for STOP mode (Wait for person)
        if blackboard.get('stop_mode', False):
            # Stop and Wait
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            # print(f"[{self.name}] Waiting for person...") # Optional logging
            return Status.RUNNING

        now = time.time()
        twist = Twist()
        
        # Helper to get average distance of a sector
        def get_avg_dist(ranges, start_deg, end_deg):
            num = len(ranges)
            start_idx = int((start_deg % 360) / 360 * num)
            end_idx = int((end_deg % 360) / 360 * num)
            if start_idx < end_idx:
                sector = ranges[start_idx:end_idx]
            else:
                sector = ranges[start_idx:] + ranges[:end_idx]
            valid = [r for r in sector if r > 0.01 and r < 10.0]
            return sum(valid) / len(valid) if valid else 0.0

        # 1. Handle Active Avoidance States
        if self.avoid_state == "BACKING":
            if now - self.avoid_start_time < 0.6: 
                twist.linear.x = -0.2
                self.pub.publish(twist)
                return Status.RUNNING
            else:
                self.avoid_state = "TURNING"
                self.avoid_start_time = now

        if self.avoid_state == "TURNING":
            if now - self.avoid_start_time < 0.5: 
                twist.linear.x = 0.0
                twist.angular.z = self.avoid_direction
                self.pub.publish(twist)
                return Status.RUNNING
            else:
                self.avoid_state = "FORWARD_SHORT"
                self.avoid_start_time = now
                
        if self.avoid_state == "FORWARD_SHORT":
            if now - self.avoid_start_time < 0.5:
                twist.linear.x = 0.2
                self.pub.publish(twist)
                return Status.RUNNING
            else:
                self.avoid_state = "IDLE"
                self.last_change_time = now 
        
        # 2. Check for Obstacles & Proactive Steering (Only if IDLE)
        if self.avoid_state == "IDLE":
            min_dist = 999.0
            if self.latest_scan:
                ranges = [r for r in self.latest_scan.ranges if r > 0.01 and r < 10.0]
                if ranges:
                    min_dist = min(ranges)
            
            # --- CRITICAL OBSTACLE (Too Close) -> BACK UP (Safety) ---
            if min_dist < 0.45:
                # Decide direction based on average side distance
                left_avg = 0
                right_avg = 0
                if self.latest_scan:
                     left_avg = get_avg_dist(self.latest_scan.ranges, 45, 135)
                     right_avg = get_avg_dist(self.latest_scan.ranges, 225, 315)
                
                if left_avg > right_avg:
                     self.avoid_direction = 1.0 # Turn Left
                else:
                     self.avoid_direction = -1.0 # Turn Right
                     
                self.avoid_state = "BACKING" # Switch to BACKING first
                self.avoid_start_time = now
                print(f"[{self.name}] CRITICAL Obstacle ({min_dist:.2f}m)! Backing up...")
                
                twist.linear.x = -0.2
                self.pub.publish(twist)
                return Status.RUNNING
            
            # --- PROACTIVE AVOIDANCE (Earlier Detection) -> Steer Away ---
            elif min_dist < 1.5:
                left_avg = get_avg_dist(self.latest_scan.ranges, 15, 75)
                right_avg = get_avg_dist(self.latest_scan.ranges, 285, 345)
                
                # Base forward motion (Slower for safer turn)
                twist.linear.x = self.speed * 0.5 
                
                # Steer towards open space (Original strength)
                steer_strength = (1.5 - min_dist) * 2.0 
                if left_avg > right_avg:
                     twist.angular.z = steer_strength # Left
                else:
                     twist.angular.z = -steer_strength # Right
                
            # --- CLEAR PATH -> Standard S-Curve ---
            else:
                if now - self.s_curve_timer > 1.5:
                    self.s_curve_dir *= -1
                    self.s_curve_timer = now
                
                twist.linear.x = self.speed
                twist.angular.z = 0.3 * self.s_curve_dir 

        self.pub.publish(twist)
        return Status.RUNNING

    def halt(self):
        twist = Twist()
        self.pub.publish(twist)


class DepthAvoidance(Node):
    def __init__(self, node_type, agent, name=None, topic="/camera/depth/image_raw", threshold=0.4, speed=0.2):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.topic = topic
        self.threshold = float(threshold) # meters
        self.speed = float(speed)
        
        self.pub = self.ros.node.create_publisher(Twist, '/cmd_vel', 10)
        
        if CvBridge:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            print(f"[{self.name}] ⚠️ CvBridge unavailable. DepthAvoidance will not function.")

        self.depth_sub = self.ros.node.create_subscription(
            Image,
            self.topic,
            self.depth_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        )
        self.latest_depth = None
        self.avoid_state = "IDLE"
        self.start_time = 0
        self.avoid_dir = 1.0

    def depth_callback(self, msg):
        self.latest_depth = msg

    async def run(self, agent, blackboard):
        if not self.bridge or self.latest_depth is None:
            return Status.FAILURE # Cannot work without data

        twist = Twist()
        now = time.time()
        
        # 1. State Machine for Active Avoidance (Back up -> Turn)
        if self.avoid_state == "BACKING":
            if now - self.start_time < 1.0:
                twist.linear.x = -0.15
                self.pub.publish(twist)
                return Status.RUNNING
            else:
                self.avoid_state = "TURNING"
                self.start_time = now
        
        elif self.avoid_state == "TURNING":
            if now - self.start_time < 1.0:
                twist.linear.x = 0.0
                twist.angular.z = self.avoid_dir * 1.5
                self.pub.publish(twist)
                return Status.RUNNING
            else:
                self.avoid_state = "IDLE"
                # Fall through to check if we are clear
                
        # 2. Check Depth for Obstacles
        if self.avoid_state == "IDLE":
            try:
                # Convert to CV2
                # Note: LIMO Astra usually '16UC1' (mm) or '32FC1' (m)
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
                
                # Normalize to meters
                if cv_image.dtype == 'uint16':
                     depth_m = cv_image.astype(float) / 1000.0
                elif cv_image.dtype == 'float32':
                     depth_m = cv_image
                else:
                     depth_m = cv_image.astype(float) # Unknown, assume m?
                
                # Check Center Region (RoI)
                h, w = depth_m.shape
                # Center 40% width, Bottom 60% height (Floor/Obstacle focus)
                y_start = int(h * 0.4)
                x_start = int(w * 0.3)
                x_end = int(w * 0.7)
                
                roi = depth_m[y_start:, x_start:x_end]
                
                # Filter zeros (invalid/too close) & max range
                valid_mask = (roi > 0.05) & (roi < 5.0)
                if np.any(valid_mask):
                    min_dist = np.min(roi[valid_mask])
                else:
                    min_dist = 10.0 # No valid data
                
                if min_dist < self.threshold:
                    print(f"[{self.name}] 🛑 Depth Obstacle Detected! ({min_dist:.2f}m < {self.threshold}m)")
                    
                    # Decide Turn Direction based on split ROI
                    roi_left = roi[:, :roi.shape[1]//2]
                    roi_right = roi[:, roi.shape[1]//2:]
                    
                    # Valid pixels average
                    l_valid = roi_left[(roi_left > 0.05) & (roi_left < 5.0)]
                    r_valid = roi_right[(roi_right > 0.05) & (roi_right < 5.0)]
                    
                    l_avg = np.mean(l_valid) if l_valid.size > 0 else 0
                    r_avg = np.mean(r_valid) if r_valid.size > 0 else 0
                    
                    # If Left is closer, turn Right (-1). If Right closer, turn Left (+1)
                    if l_avg < r_avg:
                        self.avoid_dir = -1.0
                    else:
                        self.avoid_dir = 1.0
                    
                    self.avoid_state = "BACKING"
                    self.start_time = now
                    twist.linear.x = -0.15
                    self.pub.publish(twist)
                    return Status.RUNNING
                
                else:
                    # No obstacle -> Let other nodes (Flee, Scan) run
                    return Status.FAILURE 
                    
            except Exception as e:
                print(f"[{self.name}] Error processing depth: {e}")
                return Status.FAILURE

        self.pub.publish(twist)
        return Status.RUNNING
