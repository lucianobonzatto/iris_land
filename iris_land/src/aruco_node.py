#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class ImageRepublisher:
    def __init__(self):
        # Thermal camera input and outputs. Pose must stay separate from
        # stereo/RGB ArUco so the EKF can apply different noise/gating.
        self.image_topic = rospy.get_param('~image_topic', '/thermal_camera/image_raw')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', '/flinks/image')
        self.pose_topic = rospy.get_param('~pose_topic', '/thermal/pose')
        self.frame_id = rospy.get_param('~frame_id', 'thermal_camera_frame')

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=10)
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)

        # intrínsecos/dist. (iguais ao seu arquivo)
        self.camera_matrix = np.array([[571.81579811,   0,          189.0118068],
                                       [0,         575.39583804,  134.69661555],
                                       [0        , 0        , 1   ]])

        self.distortion_coeffs = np.array([[-0.40615593,  0.15172498, -0.0043972, -0.00370684, -0.12293532]])
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.bridge = CvBridge()

        # Optional proper rotation from thermal optical frame into the frame
        # published on pose_topic. The previous x'=-y, y'=-x, z'=+z mapping
        # has determinant -1 (reflection), which is not a valid ROS rotation.
        default_R_fix = [[0, -1, 0],
                         [1,  0, 0],
                         [0,  0, 1]]
        R_fix_matrix = np.array(rospy.get_param('~axis_remap_matrix', default_R_fix), dtype=float)
        det = np.linalg.det(R_fix_matrix)
        if not np.allclose(R_fix_matrix @ R_fix_matrix.T, np.eye(3), atol=1e-3) or not np.isclose(det, 1.0, atol=1e-3):
            rospy.logerr(f"Invalid thermal axis_remap_matrix: determinant={det:.3f}. It must be a proper rotation with det=+1.")
            rospy.signal_shutdown("Invalid thermal axis remap")
        self.R_fix = R.from_matrix(R_fix_matrix)
        rospy.loginfo(f"Publishing thermal poses on {self.pose_topic} in frame {self.frame_id}")

    def image_callback(self, msg):
        # conversão fiel ao seu pipeline
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)

        # pré-processamento
        image = cv2.bilateralFilter(image, d=9, sigmaColor=75, sigmaSpace=75)
        _, image_thr = cv2.threshold(image, 60, 255, cv2.THRESH_TOZERO)

        # detecção ArUco
        corners, ids, _ = aruco.detectMarkers(image_thr, self.dictionary, parameters=self.parameters)
        n_detected = len(corners)
        rospy.loginfo_throttle(1.0, f"Thermal markers detected: {n_detected} - IDs: {ids.flatten() if ids is not None else 'none'}")

        # inicializa visualização
        image_viz = image_thr.copy()

        if n_detected > 0 and ids is not None:
            ids = ids.flatten()

            # filtra só o ID 682 (como no seu arquivo)
            indices_682 = [i for i, marker_id in enumerate(ids) if marker_id == 682]
            if indices_682:
                corners_682 = [corners[i] for i in indices_682]
                # desenha apenas os 682 para visualização
                image_viz = aruco.drawDetectedMarkers(image_viz, corners_682, ids[indices_682])

                for idx in indices_682:
                    marker = corners[idx]

                    # estima pose (rvec Rodrigues, tvec em metros)
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(marker, 0.08, self.camera_matrix, self.distortion_coeffs)
                    rvec = rvecs[0][0]  # (3,)
                    tvec = tvecs[0][0]  # (3,)

                    # ----- REMAPEAMENTO AQUI -----
                    # rotação original no frame óptico da câmera
                    R_cam = R.from_rotvec(rvec)
                    # aplica a matriz de remapeamento aos eixos (equivale a mudar de base)
                    R_new = self.R_fix * R_cam
                    quat = R_new.as_quat()  # x,y,z,w

                    # translação no novo frame
                    t_new = self.R_fix.apply(tvec)

                    # publica PoseStamped já no frame remapeado
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
                    pose_msg.header.frame_id = self.frame_id

                    pose_msg.pose.position.x = float(t_new[0])  # x' = -y (original)
                    pose_msg.pose.position.y = float(t_new[1])  # y' = -x (original)
                    pose_msg.pose.position.z = float(t_new[2])  # z' = +z

                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])

                    self.pose_pub.publish(pose_msg)
                    rospy.loginfo_throttle(1.0, f"Thermal pose ID 682: t={t_new}, q={quat}")
            else:
                rospy.logdebug_throttle(2.0, "No thermal marker ID 682 detected")
        else:
            rospy.logdebug_throttle(2.0, "No thermal marker detected")

        # publica imagem de visualização
        republished_msg = self.bridge.cv2_to_imgmsg(image_viz, encoding='rgb8')
        self.image_pub.publish(republished_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('aruco_node')
    republisher = ImageRepublisher()
    republisher.run()

