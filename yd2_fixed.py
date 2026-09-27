import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time


class YDLidarPublisher(Node):
    def __init__(self):
        super().__init__('ydlidar_python_node')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

        self.port     = '/dev/ttyUSB0'
        self.baud     = 115200
        self.frame_id = 'laser_frame'

        # mounting_offset_deg = 270
        # Diagnosis: object in FRONT  approached along GREEN (Y) axis → X/Y swapped
        #            object on  LEFT   approached along RED   (X) axis → 90° off
        # Fix: shift by (180 + 90) = 270° so lidar 90° maps to ROS forward (0°)
        self.mounting_offset_deg = 180

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            exit()

    def publish_scan(self):
        ranges = [0.0] * 360

        end_collect = time.time() + 0.09
        while time.time() < end_collect:
            b = self.ser.read(1)
            if b == b'\xaa':
                if self.ser.read(1) == b'\x55':
                    header = self.ser.read(8)
                    if len(header) < 8:
                        continue

                    ls  = header[1]
                    fsa = (header[3] << 8 | header[2]) >> 1
                    lsa = (header[5] << 8 | header[4]) >> 1

                    raw_samples = self.ser.read(ls * 2)
                    if len(raw_samples) < ls * 2:
                        continue

                    diff = (lsa + 36000 - fsa) % 36000
                    for i in range(ls):
                        dist = (raw_samples[i*2+1] << 8 | raw_samples[i*2]) / 4000.0
                        angle = (diff / (ls - 1) if ls > 1 else 0) * i + fsa
                        angle_deg = int((angle / 100.0) % 360)
                        if 0 <= angle_deg < 360:
                            ranges[angle_deg] = dist

        # REP-103 remapping:
        # index 0   = angle -π  (robot rear)
        # index 180 = angle  0  (robot forward, +X)
        # shift = (180 + mounting_offset_deg) % 360 = (180 + 270) % 360 = 90
        shift = (180 + self.mounting_offset_deg) % 360   # = 90
        reordered = ranges[shift:] + ranges[:shift]

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi
        msg.angle_increment =  math.pi / 180.0
        msg.time_increment  = 0.0
        msg.scan_time       = 0.1
        msg.range_min       = 0.1
        msg.range_max       = 8.0
        msg.ranges          = reordered

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarPublisher()
    try:
        while rclpy.ok():
            node.publish_scan()
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
