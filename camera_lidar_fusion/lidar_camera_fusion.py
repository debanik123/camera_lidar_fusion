#!/usr/bin/env python3
import math
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

class Sensor_fusion:
    def __init__(self):
        self.node = rclpy.create_node('sensor_fusion')
        self.lidar_sub = self.node.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.gpxy_sub = self.node.create_subscription(Float32, '/gpxy', self.gpxy_cb, 10)
        self.obstacle_pub = self.node.create_publisher(Bool, '/obstacle_detected', 10)
        self.uid = 9000
        self.gpx = 0.0
        self.gpy = 0.0
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        self.range_min_th  = 1.0
        self.range_max_th = 3.0
        self.min_angle = -math.pi/2
        self.max_angle = math.pi/2
        self.dis_obstracle = False
        self.obstracle_th = 0.7
        

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

        # min_index = self.angle_to_index_find(msg, -130, 'n_130_angle')
        # self.angle_to_index_find(msg, 100, 'p_100_angle')
        # self.angle_to_index_find(msg, 260, 'p_260_angle')

        # for angle in range(100, 260):
        #     (idx, range_val) = self.angle_to_index_find(msg, angle, 'obstracle')

        is_obstracle = self.is_obstacle_detected(msg, 130, 260)
        print("is_obstracle --> ", is_obstracle)

        self.publish_obstacle_detection(is_obstracle)



        

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
    
    def is_obstacle_detected(self, scan_msg, min_angle, max_angle):
        (idx, l_x, l_y, range_val) = self.angle_to_index_find(scan_msg, min_angle, 'p_min_angle_angle')
        self.publish_static_transform((l_x, l_y), 'p_min_angle_angle', 'laser_data_frame')
        (idx, l_x, l_y, range_val) = self.angle_to_index_find(scan_msg, max_angle, 'p_max_angle_angle')
        self.publish_static_transform((l_x, l_y), 'p_max_angle_angle', 'laser_data_frame')
        
        for angle in range(min_angle, max_angle):
            (idx, l_x, l_y, range_val) = self.angle_to_index_find(scan_msg, angle, 'obstracle')
            if range_val>0.2:
                if (range_val< self.obstracle_th):
                    self.publish_static_transform((l_x, l_y), 'obstracle', 'laser_data_frame')
                    return True
        
        return False
            

    
    def angle_to_index_find(self, scan_msg, angle, child_frame):
        angle = angle*(math.pi/180)
        idx = int((angle - scan_msg.angle_min)/(scan_msg.angle_increment))

        range_val = scan_msg.ranges[idx]
        angle = scan_msg.angle_min + idx * scan_msg.angle_increment

        l_y = range_val * math.sin(angle)
        l_x = range_val * math.cos(angle)

        # self.publish_static_transform((l_x, l_y), child_frame, 'laser_data_frame')
        # print(idx, l_x, l_y)
        return (idx, l_x, l_y, range_val)
    
    def publish_obstacle_detection(self, obstacle_detected):
        # Publish the obstacle detection status
        msg = Bool()
        msg.data = obstacle_detected
        self.obstacle_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = Sensor_fusion()
    rclpy.spin(lidar_subscriber.node)
    lidar_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()