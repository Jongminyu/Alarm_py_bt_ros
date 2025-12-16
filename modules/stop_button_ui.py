import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading
import sys

class StopButtonUI(Node):
    def __init__(self):
        super().__init__('stop_button_ui')
        self.publisher_ = self.create_publisher(PoseStamped, '/limo_status', 10)
        
        self.root = tk.Tk()
        self.root.title("Stop Alarm")
        self.root.geometry("300x200")
        self.root.attributes('-topmost', True) # Keep on top
        
        self.btn_stop = tk.Button(self.root, text="STOP ALARM", font=("Helvetica", 20, "bold"), bg="red", fg="white", command=self.stop_alarm)
        self.btn_stop.pack(expand=True, fill='both', padx=20, pady=20)
        
        self.running = True
        
        # Start ROS spin in separate thread
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()

    def spin_ros(self):
        try:
            from rclpy.executors import ExternalShutdownException
            while self.running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
        except (Exception, ExternalShutdownException):
            pass

    def stop_alarm(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 1.0 # Signal that button is pressed (> 0.1)
        self.publisher_.publish(msg)
        self.get_logger().info('STOP button pressed! Published signal.')
        
        # Disable button after press to indicate action taken
        self.btn_stop.config(state=tk.DISABLED, text="STOPPED", bg="gray")
        
        # Disable button temporarily
        self.btn_stop.config(state=tk.DISABLED, text="STOPPED", bg="gray")
        
        # Reset button after 5 seconds (Ready for next cycle)
        self.root.after(5000, self.reset_button)

    def reset_button(self):
        if self.running:
            self.btn_stop.config(state=tk.NORMAL, text="STOP ALARM", bg="red")

    def close(self):
        self.running = False
        if self.root:
            self.root.quit()
            self.root.destroy()
        # Thread will exit naturally

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.close()

def main():
    rclpy.init()
    ui = StopButtonUI()
    ui.run()
    # ui.run() remove duplicate
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
