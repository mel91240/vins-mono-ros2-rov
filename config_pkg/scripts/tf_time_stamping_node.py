#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
import copy

class TfTimeStampingNode(Node):
    def __init__(self):
        super().__init__('tf_time_stamping_node')
        
        self.latest_stamp = None
        
        # 1. On intercepte le vrai temps des images du Rosbag
        self.img_sub = self.create_subscription(
            Image, '/v_stereo/left/image_raw', self.image_callback, 10)
            
        # 2. On écoute directement le topic /tf global
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
            
        # 3. On republie les corrections sur le même topic /tf
        self.tf_pub = self.create_publisher(
            TFMessage, '/tf', 10)
            
        self.get_logger().info("Nœud de correction dynamique du temps TF opérationnel.")

    def image_callback(self, msg):
        self.latest_stamp = msg.header.stamp

    def tf_callback(self, msg):
        if self.latest_stamp is None:
            return
            
        modified_tf = TFMessage()
        need_to_publish = False
        
        for transform in msg.transforms:
            # Si on intercepte la liaison world -> body ET que son temps est invalide (égal à 0)
            if transform.header.frame_id == 'world' and transform.child_frame_id == 'body':
                if transform.header.stamp.sec == 0:
                    new_transform = copy.deepcopy(transform)
                    new_transform.header.stamp = self.latest_stamp
                    modified_tf.transforms.append(new_transform)
                    need_to_publish = True
            
        # On ne publie QUE si on a appliqué une correction (évite la boucle infinie)
        if need_to_publish:
            self.tf_pub.publish(modified_tf)

def main(args=None):
    rclpy.init(args=args)
    node = TfTimeStampingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
