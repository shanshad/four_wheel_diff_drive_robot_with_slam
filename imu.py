import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

# Import the ICM-20948 hardware library
from icm20948 import ICM20948

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        
        # 1. Initialize the ICM-20948 hardware
        try:
            self.imu_sensor = ICM20948(i2c_addr=0x69)
            self.get_logger().info("Successfully initialized ICM-20948 IMU sensor.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ICM-20948. Error: {e}")
            raise e

        # 2. Setup the ROS 2 Publisher
        # Topic: /imu/data, Message Type: Imu, Queue size: 10
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # 3. Setup a Timer for Data Acquisition (e.g., 50 Hz -> 0.02 seconds)
        timer_period = 0.02  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Conversion constant: Degrees to Radians
        self.deg2rad = math.pi / 180.0
        
        # Conversion constant: g-force to m/s^2 (assuming sensor library returns in g's)
        # Note: If your sensor library returns raw values, you must divide by sensitivity.
        self.g_to_mps2 = 9.80665

    def timer_callback(self):
        # Read data from the ICM-20948 hardware layer
        # read_accelerometer_gyro() returns values scaled in g-forces and degrees/sec
        try:
            ax, ay, az, gx, gy, gz = self.imu_sensor.read_accelerometer_gyro_data()
        except Exception as e:
            self.get_logger().warn(f"Failed to read data from sensor: {e}")
            return

        # Construct the standard ROS 2 Imu message
        msg = Imu()
        
        # Populate the Header frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Convert and assign Linear Acceleration (g to m/s^2)
        msg.linear_acceleration.x = ax * self.g_to_mps2
        msg.linear_acceleration.y = ay * self.g_to_mps2
        msg.linear_acceleration.z = az * self.g_to_mps2

        # Convert and assign Angular Velocity (Degrees/sec to Radians/sec)
        msg.angular_velocity.x = gx * self.deg2rad
        msg.angular_velocity.y = gy * self.deg2rad
        msg.angular_velocity.z = gz * self.deg2rad

        # The raw hardware ICM-20948 output does not provide orientation orientation.
        # Set the first element of covariance matrix to -1 to tell ROS it is invalid.
        msg.orientation_covariance[0] = -1.0

        # Publish the complete message to /imu/data
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
