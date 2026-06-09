#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import Delaunay

class VinsMeshNode(Node):
    def __init__(self):
        super().__init__('vins_mesh_node')
        # On s'abonne au topic natif de VINS
        self.sub = self.create_subscription(PointCloud, '/history_cloud', self.callback, 10)
        # On publie un Marker standard pour RViz
        self.pub = self.create_publisher(Marker, '/vins_mesh', 10)
        self.get_logger().info("Noeud de maillage VINS initialise avec succes.")

    def callback(self, msg):
        if len(msg.points) < 3:
            return

        # 1. Extraction des points x, y, z
        pts = np.array([[p.x, p.y, p.z] for p in msg.points])
        
        # 2. Triangulation 2D/3D (Delaunay sur le plan X-Y pour tendre la surface)
        tri = Delaunay(pts[:, :2])
        
        # 3. Construction du message Marker (TRIANGLE_LIST)
        marker = Marker()
        marker.header = msg.header  # On garde le frame_id 'world' et le bon timestamp
        marker.ns = "vins_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Configuration de la couleur du maillage (Vert translucide)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4
        
        # Échelle par défaut
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Pose d'origine
        marker.pose.orientation.w = 1.0

        # 4. Remplissage des sommets des triangles
        for simplex in tri.simplices:
            for idx in simplex:
                p = Point()
                p.x = float(pts[idx][0])
                p.y = float(pts[idx][1])
                p.z = float(pts[idx][2])
                marker.points.append(p)

        self.pub.publish(marker)

def main():
    rclpy.init()
    rclpy.spin(VinsMeshNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
