import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class DDSMBridge(Node):
    def __init__(self):
        super().__init__('ddsm_bridge_node')
        
        # 1. Serial setup - Match your successful test port
        self.port = '/dev/ttyACM0'
        self.baud = 115200
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit()

        # 2. Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        # 3. Parameters for your 4-wheel robot
        self.wheel_base = 0.3  # Distance between left and right wheels in meters
        self.max_rpm = 500     # Maximum speed allowed for DDSM210

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 1. Standard Differential Kinematics
        # left_vel is target speed for wheels 3 & 4
        # right_vel is target speed for wheels 1 & 2
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)

        # 2. Scale to DDSM range (-500 to 500)
        # We multiply by max_rpm to convert normalized velocity to motor units
        s_left = int(left_vel * self.max_rpm)
        s_right = int(right_vel * self.max_rpm)

        # 3. Apply Directional Mapping 
        # Based on your reference code:
        # RIGHT side (1, 2) forward is NEGATIVE (-)
        # LEFT side (3, 4) forward is POSITIVE (+)
        
        motor_right = -s_right 
        motor_left = s_left

        # 4. Clamp values to safety limits
        motor_right = max(min(motor_right, 500), -500)
        motor_left = max(min(motor_left, 500), -500)

        # 5. Format command for ESP32
        # Order: v[FrontRight],[BackRight],[FrontLeft],[BackLeft]
        command = f"v{motor_right},{motor_right},{motor_left},{motor_left}\n"
        
        self.ser.write(command.encode())

    def destroy_node(self):
        # Stop motors on shutdown
        self.ser.write(b"v0,0,0,0\n")
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DDSMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()