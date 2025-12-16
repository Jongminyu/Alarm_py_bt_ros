from modules.base_bt_nodes import BTNodeList, Status, Node
from modules.base_bt_nodes import Sequence as BaseSequence
from modules.base_bt_nodes import Fallback, ReactiveSequence, Parallel
from modules.base_bt_nodes import ReactiveFallback as BaseReactiveFallback

from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan, Image
from std_srvs.srv import Trigger
import numpy as np



# Import Actions
from scenarios.nav_demo.actions import RunOpposite, Rotate, Wander, DepthAvoidance

import random
import json
import os
import time
import math
import pygame
import tkinter as tk
from tkinter import simpledialog
from datetime import datetime

# =========================================================
# Existing Classes (Preserved)
# =========================================================

class Sequence(BaseSequence):
    def __init__(self, node_type, children, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name, children)

class ReactiveFallback(BaseReactiveFallback):
    def __init__(self, node_type, children, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name, children)

class SequenceStar(Node):
    def __init__(self, node_type, children, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.children = children
        self.current_child_index = 0

    async def run(self, agent, blackboard):
        while self.current_child_index < len(self.children):
            child = self.children[self.current_child_index]
            status = await child.run(agent, blackboard)
            
            if status == Status.RUNNING:
                return Status.RUNNING
            elif status == Status.FAILURE:
                self.current_child_index = 0 
                return Status.FAILURE
            elif status == Status.SUCCESS:
                self.current_child_index += 1
        
        self.current_child_index = 0 
        return Status.SUCCESS

class KeepRunningUntilFailure(Node):
    def __init__(self, node_type, children, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        if len(children) != 1:
            raise ValueError("KeepRunningUntilFailure must have exactly one child")
        self.child = children[0]

    async def run(self, agent, blackboard):
        status = await self.child.run(agent, blackboard)
        if status == Status.FAILURE:
            return Status.FAILURE
        return Status.RUNNING

class SaveHomePose(Node):
    def __init__(self, node_type, agent, name=None, topic_name="/amcl_pose"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.agent = agent
        self.topic_name = topic_name 
        self.saved = False
        self.cache = None
        
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.agent.ros_bridge.node.create_subscription(
            PoseWithCovarianceStamped,   
            self.topic_name,
            self._callback,
            amcl_qos  
        )

    def _callback(self, msg):
        self.cache = msg

    async def run(self, agent, blackboard):
        if self.saved:
            self.status = Status.SUCCESS
            return Status.SUCCESS

        if self.cache is not None:
            home_ps = PoseStamped()
            home_ps.header = self.cache.header
            home_ps.pose = self.cache.pose.pose
            
            blackboard['home_pose'] = home_ps
            self.agent.ros_bridge.node.get_logger().info(f"[{self.name}] Home Saved (from AMCL)")
            self.saved = True
            self.status = Status.SUCCESS
            return Status.SUCCESS
        
        self.status = Status.RUNNING
        return Status.RUNNING

class NavigateToKey(Node):
    def __init__(self, node_type, agent, bb_key_name, name=None, tolerance=0.5):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.bb_key_name = bb_key_name
        self.tolerance = float(tolerance)
        
        self.action_client = ActionClient(self.ros.node, NavigateToPose, 'navigate_to_pose')
        self.pose_sub = self.ros.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.current_pose = None
        self.goal_handle = None
        self.sent_goal = False

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    async def run(self, agent, blackboard):
        target_pose = blackboard.get(self.bb_key_name)
        if not target_pose:
            return Status.FAILURE

        # 1. Send Goal if not sent
        if not self.sent_goal:
            if not self.action_client.wait_for_server(timeout_sec=1.0):
                print(f"[{self.name}] Nav2 Action Server not available!")
                return Status.FAILURE
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = target_pose
            
            print(f"[{self.name}] Sending goal... (Tolerance: {self.tolerance}m)")
            self.goal_handle = await self.action_client.send_goal_async(goal_msg)
            
            if not self.goal_handle.accepted:
                print(f"[{self.name}] Goal rejected.")
                return Status.FAILURE
            
            # Start monitoring result
            self.result_future = self.goal_handle.get_result_async()
            self.sent_goal = True
            
            # Reset retry count if new goal
            if not hasattr(self, 'retry_count'):
                self.retry_count = 0
            
            return Status.RUNNING

        # 2. Check Action Result (Detect Plan Failure)
        if self.result_future and self.result_future.done():
            try:
                result = self.result_future.result()
                status = result.status
                
                # Check for ABORTED (4) or CANCELED (5) or SUCCEEDED (3)
                if status == GoalStatus.STATUS_ABORTED:
                    print(f"[{self.name}] Navigation ABORTED (Planner Failed?)")
                    
                    # RETRY LOGIC
                    self.retry_count += 1
                    if self.retry_count <= 3:
                         print(f"[{self.name}] Retrying... ({self.retry_count}/3)")
                         self.sent_goal = False # Trigger resend
                         return Status.RUNNING
                    else:
                         # Last ditch check: Are we close enough?
                         if self.current_pose:
                             dx = self.current_pose.position.x - target_pose.pose.position.x
                             dy = self.current_pose.position.y - target_pose.pose.position.y
                             dist = math.sqrt(dx*dx + dy*dy)
                             if dist < 1.0: # Relaxed tolerance
                                 print(f"[{self.name}] Failed but close ({dist:.2f}m). Forcing SUCCESS.")
                                 return Status.SUCCESS
                         
                         print(f"[{self.name}] Navigation Failed after retries.")
                         return Status.FAILURE

                elif status == GoalStatus.STATUS_SUCCEEDED:
                    print(f"[{self.name}] Navigation Succeeded (Server confirmed).")
                    return Status.SUCCESS
                    
            except Exception as e:
                print(f"[{self.name}] Result check error: {e}")

        # 3. Monitor Distance (Early Exit) - REMOVED to prevent premature success
        # We rely strictly on the Nav2 Action Result (Succeeded/Aborted)
        # to ensure the robot actually reaches the goal as planned.
        # if self.current_pose: ...
        
        return Status.RUNNING

    def halt(self):
        if self.goal_handle:
            # self.goal_handle.cancel_goal_async() # Fire and forget
            pass
        self.sent_goal = False
        self.result_future = None

# =========================================================
# Restored Wander & RunOpposite Nodes (Moved to actions.py)
# =========================================================

class LogDetection(Node):
    def __init__(self, node_type, agent, name=None, message="Person Detected!"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.message = message

    async def run(self, agent, blackboard):
        print(f"👁️ {self.message}")
        return Status.SUCCESS

# =========================================================
# Alarm & Utility Nodes
# =========================================================

class Wait(Node):
    def __init__(self, node_type, agent, name=None, seconds=1.0):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.seconds = float(seconds)
        self.start_time = None

    async def run(self, agent, blackboard):
        if self.start_time is None:
            self.start_time = time.time()
        
        if time.time() - self.start_time < self.seconds:
            return Status.RUNNING
        else:
            self.start_time = None # Reset for next run
            return Status.SUCCESS

class DriveForward(Node):
    def __init__(self, node_type, agent, name=None, speed=0.2, seconds=1.0):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.speed = float(speed)
        self.seconds = float(seconds)
        self.pub = self.ros.node.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None

    async def run(self, agent, blackboard):
        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Driving forward for {self.seconds}s...")
        
        twist = Twist()
        if time.time() - self.start_time < self.seconds:
            twist.linear.x = self.speed
            self.pub.publish(twist)
            return Status.RUNNING
        else:
            # Stop
            twist.linear.x = 0.0
            self.pub.publish(twist)
            self.start_time = None
            return Status.SUCCESS

# =========================================================
# Alarm & Utility Nodes
# =========================================================

class WaitUntilTime(Node):
    def __init__(self, node_type, agent, name=None, wake_time="07:00"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.target_arg = wake_time
        self.wake_time = None
        self.ui_process = None
        
        # If fixed time provided, set it immediately
        if self.target_arg != "ASK":
             self.wake_time = self.target_arg

    async def run(self, agent, blackboard):
        # 1. Ask for time if needed
        if self.wake_time is None and self.target_arg == "ASK":
            self.wake_time = self.ask_time_gui()
            if 'alarm_stopped' in blackboard:
                del blackboard['alarm_stopped']
            print(f"✅ 알람 설정 완료: [{self.wake_time}]")
            
        # 2. Check Time
        if self.wake_time:
            now = datetime.now().strftime("%H:%M")
            if now >= self.wake_time:
                # RESET for next loop (Important!)
                if self.target_arg == "ASK":
                    self.wake_time = None 
                
                # Signal new alarm cycle for other nodes to reset
                blackboard['alarm_cycle_id'] = time.time()
                return Status.SUCCESS
        
        return Status.RUNNING

    def ask_time_gui(self):
        os.environ["DISPLAY"] = ":1"
        root = tk.Tk()
        root.title("LIMO Alarm")
        root.geometry("600x400")
        root.attributes('-topmost', True)

        self.hour = 7
        self.minute = 0
        font_large = ("Arial", 40, "bold")
        font_btn = ("Arial", 20, "bold")

        frame = tk.Frame(root)
        frame.pack(expand=True, fill='both', padx=20, pady=20)
        time_label = tk.Label(frame, text=f"{self.hour:02d}:{self.minute:02d}", font=font_large)
        time_label.pack(pady=20)
        btn_frame = tk.Frame(frame)
        btn_frame.pack(pady=10)

        def update_label():
            time_label.config(text=f"{self.hour:02d}:{self.minute:02d}")
        def inc_hour():
            self.hour = (self.hour + 1) % 24
            update_label()
        def dec_hour():
            self.hour = (self.hour - 1) % 24
            update_label()
        def inc_min():
            self.minute = (self.minute + 10) % 60
            update_label()
        def dec_min():
            self.minute = (self.minute - 10) % 60
            update_label()
        def inc_min_1():
            self.minute = (self.minute + 1) % 60
            update_label()
        def dec_min_1():
            self.minute = (self.minute - 1) % 60
            update_label()

        h_frame = tk.Frame(btn_frame)
        h_frame.pack(side='left', padx=20)
        tk.Label(h_frame, text="Hour", font=("Arial", 12)).pack()
        tk.Button(h_frame, text="▲", command=inc_hour, font=font_btn, width=4, height=2).pack(pady=5)
        tk.Button(h_frame, text="▼", command=dec_hour, font=font_btn, width=4, height=2).pack(pady=5)

        m_frame = tk.Frame(btn_frame)
        m_frame.pack(side='left', padx=20)
        tk.Label(m_frame, text="Min (10)", font=("Arial", 12)).pack()
        tk.Button(m_frame, text="▲", command=inc_min, font=font_btn, width=4, height=2).pack(pady=5)
        tk.Button(m_frame, text="▼", command=dec_min, font=font_btn, width=4, height=2).pack(pady=5)
        
        m1_frame = tk.Frame(btn_frame)
        m1_frame.pack(side='left', padx=20)
        tk.Label(m1_frame, text="Min (1)", font=("Arial", 12)).pack()
        tk.Button(m1_frame, text="▲", command=inc_min_1, font=font_btn, width=4, height=2).pack(pady=5)
        tk.Button(m1_frame, text="▼", command=dec_min_1, font=font_btn, width=4, height=2).pack(pady=5)

        def on_start():
            root.destroy()

        tk.Button(root, text="START ALARM", command=on_start, font=("Arial", 20, "bold"), bg="green", fg="white", height=2).pack(fill='x', side='bottom')
        root.mainloop()
        return f"{self.hour:02d}:{self.minute:02d}"

class IsButtonPressed(ConditionWithROSTopics):
    def __init__(self, node_type, agent, name=None):
        actual_name = name if name else node_type
        topics = [(PoseStamped, '/limo_status', 'limo_status_msg')] 
        super().__init__(actual_name, agent, topics)

    def _predicate(self, agent, blackboard) -> bool:
        msg = self._cache.get('limo_status_msg')
        if msg is None:
            return False 
        if msg.pose.position.x > 0.1: 
            print("🛑 버튼 눌림 감지!")
            # Consume the message so it doesn't trigger again unless new one comes
            self._cache['limo_status_msg'] = None 
            return True
        return False
        
    async def run(self, agent, blackboard):
        if self._predicate(agent, blackboard):
            return Status.SUCCESS
        else:
            return Status.FAILURE

class IsPersonDetected(Node):
    def __init__(self, node_type, agent, name=None, output_angle="{user_angle}"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.output_key = output_angle.strip("{}")
        
        self.sub = self.ros.node.create_subscription(
            String,
            '/yolo/detection_info',
            self.callback,
            10
        )
        self.latest_msg = None
        self.last_detection_time = 0
        self.memory_duration = 0.1 # Reduced to 0.1s to prevent piling up

    def callback(self, msg):
        self.latest_msg = msg
        self.last_detection_time = time.time()

    async def run(self, agent, blackboard):
        # Stricter timing to avoid stale data from rotation
        if time.time() - self.last_detection_time < 0.2: 
            try:
                detections = json.loads(self.latest_msg.data)
                # Filter by confidence
                valid_detections = [d for d in detections if d.get('conf', 0) > 0.6]
                
                if len(valid_detections) > 0:
                    d = valid_detections[0]
                    print(f"[{self.name}] 👁️ Person Detected! (Conf: {d['conf']:.2f})")
                    return Status.SUCCESS
            except:
                pass
        return Status.FAILURE

class PlaySound(Node):
    def __init__(self, node_type, agent, name=None, file="opening.wav", loop=True):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.file_name = file
        self.loop = isinstance(loop, str) and loop.lower() == 'true' or bool(loop)
        self.audio_enabled = False
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            self.audio_enabled = True
        except Exception as e:
             print(f"⚠️ 오디오 초기화 실패 (Silent Mode): {e}")

    async def run(self, agent, blackboard):
        if blackboard.get('alarm_stopped'):
            return Status.SUCCESS
            
        if not self.audio_enabled:
            return Status.SUCCESS

        try:
            if pygame.mixer.music.get_busy():
                return Status.SUCCESS
                
            print(f"🔊 배경음(사이렌) 시작: {self.file_name}")
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sound_path = os.path.join(current_dir, self.file_name)
            
            if os.path.exists(sound_path):
                pygame.mixer.music.load(sound_path)
                loops = -1 if self.loop else 0
                pygame.mixer.music.play(loops=loops)
                pygame.mixer.music.set_volume(0.5)
            else:
                print(f"⚠️ 사운드 파일 없음: {sound_path}")
        except Exception as e:
            print(f"⚠️ 배경음 재생 오류: {e}")
            self.audio_enabled = False
            
        return Status.SUCCESS
    
class StopSound(Node):
    def __init__(self, node_type, agent, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name)

    async def run(self, agent, blackboard):
        print("🔇 알람 소리 정지")
        try:
            if pygame.mixer.get_init():
                pygame.mixer.music.stop()
        except:
             pass
        blackboard['alarm_stopped'] = True
        return Status.SUCCESS

class Speak(Node):
    def __init__(self, node_type, agent, name=None, message="good_morning.wav"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.file_name = message 
        self.played = False
        self.audio_enabled = True

    async def run(self, agent, blackboard):
        if self.played:
            return Status.SUCCESS
            
        if not self.audio_enabled: 
            return Status.SUCCESS

        try:
            if not pygame.mixer.get_init():
                try:
                    pygame.mixer.init()
                except:
                    self.audio_enabled = False
                    return Status.SUCCESS

            if pygame.mixer.music.get_busy():
                return Status.RUNNING
                
            if not hasattr(self, 'started'):
                self.started = False
                
            if not self.started:
                print(f"🗣️ 엔딩 멘트 재생: {self.file_name}")
                current_dir = os.path.dirname(os.path.abspath(__file__))
                sound_path = os.path.join(current_dir, self.file_name)
                
                if os.path.exists(sound_path):
                    pygame.mixer.music.load(sound_path)
                    pygame.mixer.music.set_volume(1.0)
                    pygame.mixer.music.play()
                    self.started = True
                    return Status.RUNNING
                else:
                    self.played = True
                    return Status.SUCCESS
            else:
                if pygame.mixer.music.get_busy():
                    return Status.RUNNING
                else:
                    self.played = True
                    return Status.SUCCESS
                    
        except Exception as e:
            print(f"⚠️ Speak 오류 (Pass): {e}")
            self.played = True
            return Status.SUCCESS



class CaptureImage(Node):
    def __init__(self, node_type, agent, name=None, topic="/camera/color/image_raw", delay=0.0):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.delay = float(delay)
        self.start_time = None
        
        # Create Service Client
        self.cli = self.ros.node.create_client(Trigger, 'capture_image_service')
        self.req_future = None

    async def run(self, agent, blackboard):
        # 1. Handle Delay
        if self.delay > 0:
            if self.start_time is None:
               self.start_time = time.time()
               print(f"[{self.name}] Waiting {self.delay}s before capture...")
            
            if time.time() - self.start_time < self.delay:
                return Status.RUNNING
        
        # 2. Check Service
        if not self.cli.wait_for_service(timeout_sec=1.0):
            print(f"[{self.name}] Camera Service not available!")
            return Status.FAILURE
            
        # 3. Call Service
        if self.req_future is None:
            req = Trigger.Request()
            self.req_future = self.cli.call_async(req)
            print(f"[{self.name}] Requesting Photo Capture...")
            return Status.RUNNING
            
        # 3. Check Result
        if self.req_future.done():
            try:
                response = self.req_future.result()
                if response.success:
                    print(f"📸 SUCCESS: {response.message}")
                    self.req_future = None
                    return Status.SUCCESS
                else:
                    print(f"⚠️ FAIL: {response.message}")
                    self.req_future = None
                    return Status.FAILURE
            except Exception as e:
                 print(f"[{self.name}] Service Call Exception: {e}")
                 self.req_future = None
                 return Status.FAILURE
        
        return Status.RUNNING

class ClearCostmap(Node):
    def __init__(self, node_type, agent, name=None, service_name="/global_costmap/clear_entirely_global_costmap"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.ros = agent.ros_bridge
        self.service_name = service_name
        self.cli = self.ros.node.create_client(Trigger, self.service_name)
        self.req_future = None

    async def run(self, agent, blackboard):
        if not self.cli.wait_for_service(timeout_sec=0.5):
            print(f"[{self.name}] Service {self.service_name} not available (Skip).")
            return Status.SUCCESS 
            
        if self.req_future is None:
            req = Trigger.Request()
            self.req_future = self.cli.call_async(req)
            print(f"[{self.name}] Clearing Costmap...")
            return Status.RUNNING
            
        if self.req_future.done():
            try:
                self.req_future.result()
                print(f"[{self.name}] Costmap Cleared! Data Reset.")
            except Exception as e:
                print(f"[{self.name}] Failed to clear: {e}")
            self.req_future = None
            return Status.SUCCESS
            
        return Status.RUNNING

# Register Nodes
BTNodeList.CONTROL_NODES.extend([
    'Sequence',
    'Fallback',
    'ReactiveSequence',
    'ReactiveFallback',
    'Parallel',
    'KeepRunningUntilFailure',
    'SequenceStar'
])

BTNodeList.ACTION_NODES.extend([
    'SaveHomePose',
    'NavigateToKey',
    'WaitUntilTime',
    'PlaySound',
    'StopSound',
    'RunOpposite',
    'Wander',
    'Speak',

    'LogDetection',
    'Rotate',
    'CaptureImage',
    'Wait',
    'DriveForward',
    'ClearCostmap',
    'DepthAvoidance'
])

BTNodeList.CONDITION_NODES.extend([
    'IsButtonPressed',
    'IsPersonDetected'
])
