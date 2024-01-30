import math
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Sensor_fusion:
    def __init__(self):
        self.node = rclpy.create_node('sensor_fusion')
        self.lidar_sub = self.node.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.gpxy_sub = self.node.create_subscription(Float32, '/float32_topic', self.gpxy_cb, 10)
        self.uid = 9000
        self.gpx = 0.0
        self.gpy = 0.0
    
    def gpxy_cb(self, msg):
        self.uid = msg.data[0]
        self.gpx = msg.data[1]
        self.gpy = msg.data[2]



    def lidar_cb(self, msg):
        print("Received LIDAR data:")
        print("Ranges:", len(msg.ranges))
        print("Intensities:", len(msg.intensities))

        min_l_xy = self.cor_lidar_cam(msg, self.gpx, self.gpy)
    

    def cor_lidar_cam(self, scan_msg, x, y):
        ranges = []
        l_xy = []
        for i in range(len(scan_msg.ranges)):
            range_val = scan_msg.ranges[i]
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            l_y = range_val * math.sin(angle)
            l_x = range_val * math.cos(angle)

            if math.isfinite(l_y) and math.isfinite(l_x):
                if 0.5 < range_val < 3.0:
                    ranges.append(math.sqrt((l_x - x)**2 + (l_y - y)**2))
                    l_xy.append((l_x, l_y))
                else:
                    ranges.append(float('inf'))
                    l_xy.append((float('inf'), float('inf')))
            else:
                ranges.append(float('inf'))
                l_xy.append((float('inf'), float('inf')))

        min_value, min_index = min((value, index) for index, value in enumerate(ranges))
        min_l_xy = l_xy[min_index]
        return min_l_xy


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Sensor_fusion()
    rclpy.spin(lidar_subscriber.node)
    lidar_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
