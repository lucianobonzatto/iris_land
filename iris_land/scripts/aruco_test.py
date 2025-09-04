#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisherAndPoseSubscriber(Node):
    def __init__(self, image_path):
        super().__init__('image_pub_pose_sub_node')
        self.bridge = CvBridge()
        self.rotation_angle = 0 

        # Publicador de imagem
        self.image_pub = self.create_publisher(Image, '/iris/usb_cam/image_raw', 10)

        # Subscritor de pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.pose_callback,
            10
        )

        # Carrega a imagem do disco
        if not os.path.isfile(image_path):
            self.get_logger().error(f"Arquivo não encontrado: {image_path}")
            return

        self.cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if self.cv_image is None:
            self.get_logger().error(f"Falha ao ler a imagem: {image_path}")
            return

        # Publica a imagem a cada 0.1s (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_image)

    def publish_image(self):
        try:

            if self.rotation_angle != 0:
                (h, w) = self.cv_image.shape[:2]
                center = (w // 2, h // 2)
                M = cv2.getRotationMatrix2D(center, self.rotation_angle, 1.0)
                cv_image_rot = cv2.warpAffine(self.cv_image, M, (w, h))
            else:
                cv_image_rot = self.cv_image.copy()

                
            image_msg = self.bridge.cv2_to_imgmsg(cv_image_rot, encoding='bgr8')
            self.image_pub.publish(image_msg)

            self.rotation_angle = self.rotation_angle + 1

            print(self.rotation_angle)
        except Exception as e:
            self.get_logger().error(f"Erro ao publicar imagem: {e}")

    def pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation

        import numpy as np
        from scipy.spatial.transform import Rotation as R
        quat = [ori.x, ori.y, ori.z, ori.w]
        euler = R.from_quat(quat).as_euler('ZYX', degrees=True)

        # Exibindo informações úteis
        self.get_logger().info(
            f"Pose recebida ->\n"
            f"Posição (x, y, z): ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) |\n"
            f"Orientação (quaternion x, y, z, w): ({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})\n"
            f"Euler ZYX (Yaw, Pitch, Roll) em graus: ({euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f})\n\n"
        )

def main(args=None):
    rclpy.init(args=args)

    image_path = '/home/lukn23/ros2_ws/src/iris_land/frame_43.jpg'
    image_path = '/home/lukn23/ros2_ws/src/iris_land/frame_4.jpg'
    image_path = '/home/lukn23/ros2_ws/src/iris_land/frame_ex.jpg'
    node = ImagePublisherAndPoseSubscriber(image_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
