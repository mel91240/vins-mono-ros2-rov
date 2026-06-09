import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CamInfoBridge(Node):
    def __init__(self):
        super().__init__('talker_camera_info')
        self.pub = self.create_publisher(CameraInfo, '/alphasense_driver_ros/cam0/camera_info', 10)
        self.sub = self.create_subscription(Image, '/alphasense_driver_ros/cam0', self.image_callback, 10)

    def image_callback(self, msg):
        info = CameraInfo()
        info.header = msg.header  # Synchronisation temporelle parfaite
        info.width = 720
        info.height = 540
        info.distortion_model = 'equidistant'
        info.d = [0.04816514299784573, 0.1707359987229406, -0.30080163056061726, 0.47220233022405056]
        info.k = [460.1163446727443, 0.0, 357.8825573770236,
                  0.0, 460.4372977453345, 266.43728860156045,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [460.1163446727443, 0.0, 357.8825573770236, 0.0,
                  0.0, 460.4372977453345, 266.43728860156045, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub.publish(info)

def main():
    rclpy.init()
    node = CamInfoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
