import rclpy
from sensor_msgs.msg import LaserScan

class Sensor_fusion:
    def __init__(self):
        self.node = rclpy.create_node('sensor_fusion')
        self.lidar_sub = self.node.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)

    def lidar_cb(self, msg):
        print("Received LIDAR data:")
        print("Ranges:", len(msg.ranges))
        print("Intensities:", len(msg.intensities))

        angle_max = msg.angle_max
        print("angle_max", angle_max)


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Sensor_fusion()
    rclpy.spin(lidar_subscriber.node)
    lidar_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
