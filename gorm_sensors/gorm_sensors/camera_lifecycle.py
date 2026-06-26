
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os
import signal

class RestartNode(Node):
    def __init__(self):
        super().__init__('camera_restart_node')
        self.subscription = self.create_subscription(
            Empty,
            '/restart_cameras',
            self.restart_callback,
            10
        )
        self.launch_process = None
        self.start_launch()

    def start_launch(self):
        self.get_logger().info('Starting camera launch...')
        self.launch_process = subprocess.Popen(
            ['ros2', 'launch', 'gorm_sensors', 'cameras_gnss.launch.py'],
            preexec_fn=os.setsid
        )

    def restart_callback(self, msg):
        self.get_logger().info('Restart signal received. Restarting cameras...')
        if self.launch_process:
            os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
            self.launch_process.wait()
        self.start_launch()

def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = RestartNode()
    rclpy.spin(lifecycle_node)
    lifecycle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
