#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from collections import deque
import copy

class VirtualStereoNode(Node):
    def __init__(self):
        super().__init__('virtual_stereo_node')
        
        # Buffer pour mémoriser les images et créer le décalage temporel
        self.image_buffer = deque(maxlen=3) # 3 frames de décalage (environ 150ms)
        
        # Abonnement au flux monoculaire réel du bag
        self.img_sub = self.create_subscription(Image, '/cam0/image_raw', self.image_callback, 10)
        
        # Éditeurs pour la stéréo virtuelle
        self.left_img_pub = self.create_publisher(Image, '/v_stereo/left/image_raw', 10)
        self.right_img_pub = self.create_publisher(Image, '/v_stereo/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/v_stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/v_stereo/right/camera_info', 10)
        
        # Définition de la calibration fixe EuRoC (cam0)
        self.base_info = CameraInfo()
        self.base_info.height = 480
        self.base_info.width = 752
        self.base_info.distortion_model = "plumb_bob"
        self.base_info.d = [-0.28340811, 0.07395914, 0.00019359, 1.76187114e-05, 0.0]
        self.base_info.k = [458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0]
        self.base_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        self.get_logger().info("Nœud de stéréoscopie temporelle prêt.")

    def image_callback(self, msg):
        # On stocke l'image reçue dans le buffer
        self.image_buffer.append(copy.deepcopy(msg))
        
        # On attend que le buffer soit plein pour commencer à publier
        if len(self.image_buffer) < self.image_buffer.maxlen:
            return
            
        # Image gauche = L'image actuelle (la plus récente, à la fin du buffer)
        left_img = copy.deepcopy(self.image_buffer[-1])
        # Image droite = L'image passée (la plus ancienne, au début du buffer)
        right_img = copy.deepcopy(self.image_buffer[0])
        
        # CRUCIAL : On force l'image passée à avoir le MEME timestamp que l'actuelle
        right_img.header.stamp = left_img.header.stamp
        
        left_img.header.frame_id = "camera"
        right_img.header.frame_id = "camera"
        
        # Préparation des CameraInfo avec le même timestamp courant
        left_info = copy.deepcopy(self.base_info)
        left_info.header.stamp = left_img.header.stamp
        left_info.header.frame_id = "camera"
        left_info.p = [458.654, 0.0, 367.215, 0.0, 0.0, 457.296, 248.375, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        right_info = copy.deepcopy(self.base_info)
        right_info.header.stamp = left_img.header.stamp
        right_info.header.frame_id = "camera"
        right_info.p = [458.654, 0.0, 367.215, -45.8654, 0.0, 457.296, 248.375, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # Publication simultanée
        self.left_img_pub.publish(left_img)
        self.right_img_pub.publish(right_img)
        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
