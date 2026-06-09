#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Abonnement à l'image brute pour récupérer le timing exact
        self.image_sub = self.create_subscription(
            Image, '/cam0/image_raw', self.image_callback, 10)
            
        # Editeur du CameraInfo synchronisé
        self.info_pub = self.create_publisher(
            CameraInfo, '/cam0/camera_info', 10)
            
        self.get_logger().info("Nœud CameraInfo injecteur pour EuRoC démarré.")

    def image_callback(self, msg):
        info_msg = CameraInfo()
        info_msg.header = msg.header  # On copie EXACTEMENT le même timestamp et frame_id
        info_msg.header.frame_id = "camera"
        # Dimensions de l'image EuRoC
        info_msg.height = 480
        info_msg.width = 752
        
        # Modèle de distorsion (Plumb Bob / Pin-hole classique)
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [-1.791320e-01, 1.481230e-01, 3.230000e-04, -4.730000e-04, 0.0]
        
        # Matrice Intrinsèque K [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        info_msg.k = [461.6, 0.0, 363.0,
                      0.0, 460.3, 248.1,
                      0.0, 0.0, 1.0]
                      
        # Matrice de Rectification R (Identité par défaut)
        info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
                      
        # Matrice de Projection P [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
        info_msg.p = [461.6, 0.0, 363.0, 0.0,
                      0.0, 460.3, 248.1, 0.0,
                      0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
