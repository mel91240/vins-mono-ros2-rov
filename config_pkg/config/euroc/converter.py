#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2

class CloudConverter(Node):
    def __init__(self):
        super().__init__('cloud_converter_node')
        self.sub = self.create_subscription(PointCloud, '/point_cloud', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/vins_pointcloud2', 10)

    def callback(self, msg):
        # On définit 4 champs : x, y, z ET intensity pour satisfaire RTAB-Map
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]
        
        # On ajoute une intensité par défaut (1.0) à chaque point x,y,z
        points_list = [[p.x, p.y, p.z, 1.0] for p in msg.points]

        out_msg = point_cloud2.create_cloud(msg.header, fields, points_list)
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    rclpy.spin(CloudConverter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
