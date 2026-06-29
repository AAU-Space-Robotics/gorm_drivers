#!/usr/bin/env python3
import os
import signal
import Jetson.GPIO as GPIO
import rclpy
import canopen
import atexit
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


# =========================
# CONSTANTS
# =========================

# Motor IDs
FRONT_LEFT = 1
FRONT_RIGHT = 2
CENTER_LEFT = 3
CENTER_RIGHT = 4
REAR_LEFT = 5
REAR_RIGHT = 6
FRONT_LEFT_ANGLE = 7
FRONT_RIGHT_ANGLE = 8
REAR_LEFT_ANGLE = 9
REAR_RIGHT_ANGLE = 10

MOTOR_IDS = [
    FRONT_LEFT, FRONT_RIGHT, CENTER_LEFT,
    CENTER_RIGHT, REAR_LEFT, REAR_RIGHT,
    FRONT_LEFT_ANGLE, FRONT_RIGHT_ANGLE,
    REAR_LEFT_ANGLE, REAR_RIGHT_ANGLE
]

# ✅ SENSOR BIT (du fandt: bit 18)
SENSOR_BIT = 262144

# ✅ OFFSET (justér senere!)
HOME_OFFSETS = {
    7: 1500,
    8: 1500,
    9: 1500,
    10: 1500
}


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')
        self._initialize_motor_parameters()

        self.network = canopen.Network()
        self.network.connect(channel='can1', bustype='socketcan')

        GPIO.setmode(GPIO.BOARD)

        # Init nodes
        for index in self.motor_ids:
            self.network.add_node(index, self.eds_path)

        # Sub
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/motor_commands',
            self.listener_callback,
            10)

        # Services
        self.start_motors_service = self.create_service(
            Trigger, 'start_motors', self.start_motors_callback)

        self.shutdown_motors_service = self.create_service(
            Trigger, 'shutdown_motors', self.shutdown_motors_callback)

        # ✅ HOMING SERVICE
        self.home_service = self.create_service(
            Trigger, 'home_steering', self.home_callback)

        self.configure_motor_settings()
        self.start_motors()

        atexit.register(self.on_shutdown)

    # =========================
    # CALLBACKS
    # =========================

    def listener_callback(self, msg):
        steering_scale = 1000
        velocity_scale = 315 * 1/4

        for idx, node_id in enumerate(self.network):
            node = self.network[node_id]

            if node_id < 7:
                node.sdo['vl target velocity'].phys = msg.data[idx] * velocity_scale
            else:
                node.sdo['Controlword'].bits[4] = 0
                node.sdo[0x607A].phys = msg.data[idx] * steering_scale
                node.sdo['Controlword'].bits[5] = 1
                node.sdo['Controlword'].bits[4] = 1

    def start_motors_callback(self, request, response):
        self.start_motors()
        response.success = True
        response.message = "Motors started"
        return response

    def shutdown_motors_callback(self, request, response):
        self.shutdown_motors()
        response.success = True
        response.message = "Motors stopped"
        return response

    def home_callback(self, request, response):
        self.get_logger().info("Starting homing...")

        # ✅ safety check
        for node_id in self.steering_motor_ids:
            node = self.network[node_id]
            if node.sdo[0x60FD].raw & SENSOR_BIT:
                response.success = False
                response.message = f"Motor {node_id} already on sensor"
                return response

        self.home_all_steering()

        response.success = True
        response.message = "Homing complete"
        return response

    def on_shutdown(self):
        self.shutdown_motors()
        self.network.disconnect()

    # =========================
    # SETUP
    # =========================

    def _initialize_motor_parameters(self):
        self.motor_ids = MOTOR_IDS
        self.velocity_motor_ids = MOTOR_IDS[:6]
        self.steering_motor_ids = MOTOR_IDS[6:]
        self.eds_path = 'install/gorm_base_control/share/gorm_base_control/config/C5-E-2-09.eds'

    def configure_motor_settings(self):
        for node_id in self.network:
            node = self.network[node_id]

            if node_id < 7:
                node.sdo['Modes of operation'].phys = 0x02
            else:
                node.sdo['Modes of operation'].phys = 0x01
                node.sdo[0x6081].phys = 200

    # =========================
    # MOTOR CONTROL
    # =========================

    def start_motors(self):
        for node_id in self.network:
            node = self.network[node_id]
            node.sdo['Controlword'].phys = 0x0006
            node.sdo['Controlword'].phys = 0x0007
            node.sdo['Controlword'].phys = 0x000F

    def shutdown_motors(self):
        for node_id in self.network:
            node = self.network[node_id]
            node.sdo['Controlword'].phys = 0x0000

    # =========================
    # ✅ HOMING LOGIC
    # =========================

    def home_all_steering(self):
        for node_id in self.steering_motor_ids:
            self.home_one_motor(node_id)

    def home_one_motor(self, node_id):
        node = self.network[node_id]
        offset = HOME_OFFSETS[node_id]

        self.get_logger().info(f"Homing motor {node_id}")

        # Enable
        node.sdo['Controlword'].phys = 0x0006
        node.sdo['Controlword'].phys = 0x0007
        node.sdo['Controlword'].phys = 0x000F

        # --- Fast forward ---
        node.sdo[0x6081].phys = 200
        node.sdo[0x607A].phys = 1000000
        node.sdo['Controlword'].phys = 0x001F

        while not (node.sdo[0x60FD].raw & SENSOR_BIT):
            time.sleep(0.001)

        node.sdo['Controlword'].phys = 0x0000

        # --- Back off ---
        node.sdo[0x607A].phys = -3000
        node.sdo['Controlword'].phys = 0x001F

        while node.sdo[0x60FD].raw & SENSOR_BIT:
            time.sleep(0.001)

        node.sdo['Controlword'].phys = 0x0000

        # --- Slow approach ---
        node.sdo[0x6081].phys = 50
        node.sdo[0x607A].phys = 3000
        node.sdo['Controlword'].phys = 0x001F

        while not (node.sdo[0x60FD].raw & SENSOR_BIT):
            time.sleep(0.001)

        node.sdo['Controlword'].phys = 0x0000

        # --- Move to offset ---
        node.sdo[0x607A].phys = offset
        node.sdo['Controlword'].phys = 0x001F

        time.sleep(0.3)

        # --- Set home ---
        node.sdo[0x6064].phys = 0

        self.get_logger().info(f"Motor {node_id} homed ✅")


# =========================
# MAIN
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
