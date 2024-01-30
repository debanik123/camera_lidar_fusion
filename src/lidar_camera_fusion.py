import math
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Sensor_fusion:
    def __init__(self):
        self.node = rclpy.create_node('sensor_fusion')
        self.lidar_sub = self.node.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.gpxy_sub = self.node.create_subscription(Float32, '/gpxy', self.gpxy_cb, 10)
        self.uid = 9000
        self.gpx = 0.0
        self.gpy = 0.0
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        self.range_min_th  = 1.0
        self.range_max_th = 3.0
        self.min_angle = -math.pi/2
        self.max_angle = math.pi/2
        self.dis_obstracle = False

        

    def gpxy_cb(self, msg):
        self.uid = msg.data[0]
        self.gpx = msg.data[1]
        self.gpy = msg.data[2]



    def lidar_cb(self, msg):
        print("Received LIDAR data:")
        print("Ranges:", len(msg.ranges))
        print("Intensities:", len(msg.intensities))
        # self.is_obstacle_detected_  = self.is_obstacle_detected(msg)

        # print("is_obstacle_detected_ -> ",self.is_obstacle_detected_)


        # min_l_xy = self.cor_lidar_cam(msg, self.gpx, self.gpy)
        # print(min_l_xy)
        # self.publish_static_transform(min_l_xy, 'min_point', 'laser_data_frame')

        self.angle_to_index_find(msg, math.pi/3)

    def cor_lidar_cam(self, scan_msg, x, y):
        ranges = []
        l_xy = []
        for i in range(len(scan_msg.ranges)):
            range_val = scan_msg.ranges[i]
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            l_y = range_val * math.sin(angle)
            l_x = range_val * math.cos(angle)

            if math.isfinite(l_y) and math.isfinite(l_x):
                if self.range_min_th < range_val < self.range_max_th:
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
    
    def publish_static_transform(self, min_l_xy, child_frame, parent_frame):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = parent_frame # Replace with your lidar frame_id
        static_transformStamped.child_frame_id = child_frame
        static_transformStamped.transform.translation.x = min_l_xy[0]
        static_transformStamped.transform.translation.y = min_l_xy[1]
        static_transformStamped.transform.translation.z = 0.0
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([static_transformStamped])
    
    def is_obstacle_detected(self, lidar_msg):
        min_index = max(0, math.floor((self.min_angle - lidar_msg.angle_min) / lidar_msg.angle_increment))
        max_index = min(len(lidar_msg.ranges) - 1, math.ceil((self.max_angle - lidar_msg.angle_min) / lidar_msg.angle_increment))

        for i in range(min_index, max_index + 1):
            range_val = lidar_msg.ranges[i]
            angle = lidar_msg.angle_min + i * lidar_msg.angle_increment
            
            l_y = range_val * math.sin(angle)
            l_x = range_val * math.cos(angle)

            if 1.0<range_val <1.5:
                self.publish_static_transform((l_x, l_y), 'min_point', 'laser_data_frame')

            if 1.0<lidar_msg.ranges[i] < 1.5:
                return True  # Obstacle detected

        return False
    
    def angle_to_index_find(self, scan_msg, angle):
        idx = int((angle - scan_msg.angle_min)/(scan_msg.angle_increment))

        range_val = scan_msg.ranges[idx]
        angle = scan_msg.angle_min + idx * scan_msg.angle_increment

        l_y = range_val * math.sin(angle)
        l_x = range_val * math.cos(angle)

        self.publish_static_transform((l_x, l_y), 'min_point', 'laser_data_frame')
        print(idx, l_x, l_y)


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Sensor_fusion()
    rclpy.spin(lidar_subscriber.node)
    lidar_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
