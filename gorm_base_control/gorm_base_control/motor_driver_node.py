#!/usr/bin/env python3
import os
import signal
import Jetson.GPIO as GPIO
import rclpy
import canopen
import atexit

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

## Constants

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

MOTOR_IDS = [   FRONT_LEFT, FRONT_RIGHT, CENTER_LEFT,
                CENTER_RIGHT, REAR_LEFT, REAR_RIGHT,
                FRONT_LEFT_ANGLE, FRONT_RIGHT_ANGLE,
                REAR_LEFT_ANGLE, REAR_RIGHT_ANGLE] 


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')
        self._initialize_motor_parameters()
        # Initialize can network
        self.network = canopen.Network()
        self.network.connect(channel='can1', bustype='socketcan')
        GPIO.setmode(GPIO.BOARD)
        self.get_logger().info('Got here')
        self.motor_operation_mode = []
        self.control_word = []

        # initialize nodes
        for index in self.motor_ids:
            try:
                self.network.add_node(index, self.eds_path)
                # self.motor_operation_mode.append(self.node.sdo['Modes of operation'])
                # self.control_word.append(self.node.sdo['Controlword'])
            except Exception as e:
                self.get_logger().error(f"Failed to initialize motor {index}: {e}")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/motor_commands', # TODO - Change this to the correct topic
            self.listener_callback,
            10)

        self.start_motors_service = self.create_service(
            Trigger,
            'start_motors',
            self.start_motors_callback)
        self.shutdown_motors_service = self.create_service(
            Trigger,
            'shutdown_motors',
            self.shutdown_motors_callback)

        self.configure_motor_settings()
        self.start_motors()

        # Register shutdown callback
        # rclpy.get_default_context().on_shutdown(self.on_shutdown)
        atexit.register(self.on_shutdown)

    # =================================================================================
    # ROS 2 Callbacks
    # =================================================================================
    def listener_callback(self, msg: Float64MultiArray):
        '''Callback function for the subscriber'''
        steering_scale = 1000
        velocity_scale = 315 * 1/4
        motor_commands = msg.data
        try:
            
            for idx, node_id in enumerate(self.network):
                node = self.network[node_id]
                if node_id < 7:
                    node.sdo['vl target velocity'].phys = motor_commands[idx] * velocity_scale
                else:
                    node.sdo['Controlword'].bits[4] = 0
                    node.sdo[0x607A].phys = motor_commands[idx] * steering_scale
                    node.sdo['Controlword'].bits[5] = 1
                    node.sdo['Controlword'].bits[4] = 1

        except Exception as e:
            self.get_logger().error(f"Failed in listener callback: {e}")

    def start_motors_callback(self, request, response):
        '''Callback for the start_motors service'''
        self.get_logger().info("Service call to start motors...")
        try:
            self.start_motors()
            response.success = True
            response.message = "Motors started successfully."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to start motors: {e}"
            self.get_logger().error(response.message)
        return response

    def shutdown_motors_callback(self, request, response):
        '''Callback for the shutdown_motors service'''
        self.get_logger().info("Service call to stop motors...")
        try:
            self.shutdown_motors()
            response.success = True
            response.message = "Motors stopped successfully."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop motors: {e}"
            self.get_logger().error(response.message)
        return response

    def on_shutdown(self):
        '''Callback function for shutdown'''
        self.get_logger().info('Shutting down motors')
        self.shutdown_motors()
        self.network.disconnect()

    # =================================================================================
    # Setup & Configuration Methods
    # =================================================================================
    def _initialize_motor_parameters(self):
        # Max linear and angular velocities
        self.max_linear_vel = 2000
        self.max_angular_vel = 400

        # Motor IDs
        self.motor_ids = MOTOR_IDS
        self.velocity_motor_ids = MOTOR_IDS[:6]
        self.steering_motor_ids = MOTOR_IDS[6:]

        # Target velocity and position
        self.target_position = [0] * len(self.motor_ids)

        # EDS path
        self.eds_path = 'install/gorm_base_control/share/gorm_base_control/config/C5-E-2-09.eds'

    def configure_motor_settings(self):
        # Set up velocity motors settings
        for node_id in self.network:
            node = self.network[node_id]
            if node_id < 7:
                node.sdo['Modes of operation'].phys = 0x02
                
                node.sdo['vl velocity acceleration']['DeltaSpeed'].phys = 3500
                node.sdo['vl velocity acceleration']['DeltaTime'].phys = 2

                node.sdo[0x6049][0x01].phys = 3500
                node.sdo[0x6049][0x02].phys = 2 

                node.sdo[0x6046][0x02].phys = 1500             # TODO - Document what this is min max speed
                node.sdo[0x60A9].raw   = 0x00B44700

                
            else:
                node.sdo['Modes of operation'].phys = 0x01
                node.sdo[0x6081].phys = 200 # Profile Velocity
                node.sdo[0x6083].phys = 350 # Profile Acceleration
                node.sdo[0x6084].phys = 350 # Profile Deceleration

                node.sdo[0x60C5].phys = 700 # Max Acceleration
                node.sdo[0x60C6].phys = 700 # Max Deceleration

                node.sdo[0x6091][0x01].phys = 62
                node.sdo[0x6091][0x02].phys = 1

                node.sdo['Controlword'].phys = 0x0006
                node.sdo[0x60A8].raw   = 0xFD100000

    # =================================================================================
    # Motor Control Methods
    # =================================================================================
    def start_motors(self):
        for node_id in self.network:
            node = self.network[node_id]
            node.sdo['Controlword'].phys = 0x0006 # 0110, 
            self.get_logger().info('Set Controlword to 0x0006')
            if node.sdo['Statusword'].bits[0] == 1 and node.sdo['Statusword'].bits[5] == 1 and node.sdo['Statusword'].bits[9] == 1:
                node.sdo['Controlword'].phys = 0x0007 # 0111 
                self.get_logger().info('Set Controlword to 0x0007')
                if node.sdo['Statusword'].bits[0] == 1 and node.sdo['Statusword'].bits[1] == 1 and node.sdo['Statusword'].bits[4] == 1 and node.sdo['Statusword'].bits[5] == 1 and node.sdo['Statusword'].bits[9]:
                    # bit0: SO (Switched On), bit1: EV (Enable Voltage), bit2: QS (Quick Stop), bit3: EO (Enable Operation)
                    node.sdo['Controlword'].phys = 0x000F # 1111 
                    self.get_logger().info('Set Controlword to 0x000F')
                else:
                    self.get_logger().info('Failed to set Controlword to 0x000F')
            else:
                self.get_logger().info('Failed to set Controlword to 0x0007')
        
        # Send zero commands to all motors upon startup
        self.send_zero_commands()
    
    def shutdown_motors(self):
        '''Shutdown the motors by transitioning through the CiA 402 Power State Machine.'''
        try:
            self.get_logger().info('Shutting down motors...')
            for node_id in self.network:
                node = self.network[node_id]
                # Send the "Shutdown" command (0x0006) to transition from "Operation Enabled" to "Ready to Switch On"
                node.sdo['Controlword'].phys = 0x0006
            self.get_logger().info("Sent 'Shutdown' command (0x0006) to all motors.")

            for node_id in self.network:
                node = self.network[node_id]
                # Send the "Disable Voltage" command (0x0000) to transition to "Switch on Disabled"
                node.sdo['Controlword'].phys = 0x0000
            self.get_logger().info("Sent 'Disable Voltage' command (0x0000) to all motors.")
            
            self.get_logger().info('Motor shutdown sequence complete.')

        except Exception as e:
            self.get_logger().error(f"Failed to properly shut down motors: {e}")
    
    def send_zero_commands(self):
        '''Send zero velocity/position commands to all motors for safe startup'''
        self.get_logger().info('Sending zero commands to all motors...')
        success_count = 0
        
        for node_id in self.network:
            try:
                node = self.network[node_id]
                if node_id < 7:  # Velocity motors (1-6)
                    node.sdo['vl target velocity'].phys = 0
                    self.get_logger().info(f'Sent zero velocity command to motor {node_id}')
                else:  # Position motors (7-10)
                    # For position motors, we set the target position to current position (0)
                    node.sdo['Controlword'].bits[4] = 0
                    node.sdo[0x607A].phys = 0  # Target position = 0
                    node.sdo['Controlword'].bits[5] = 1
                    node.sdo['Controlword'].bits[4] = 1
                    self.get_logger().info(f'Sent zero position command to motor {node_id}')
                
                success_count += 1
            except Exception as e:
                self.get_logger().error(f"Failed to send zero command to motor {node_id}: {e}")
        
        self.get_logger().info(f'Zero commands sent to {success_count}/{len(self.network)} motors.')
    

def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    try:
        rclpy.spin(motor_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver_node.on_shutdown()
        motor_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
