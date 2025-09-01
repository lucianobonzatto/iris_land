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
        # tópicos da sua térmica e saídas
        self.image_sub = rospy.Subscriber('/thermal_camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/flinks/image', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('/aruco/pose', PoseStamped, queue_size=10)

        # intrínsecos/dist. (iguais ao seu arquivo)
        self.camera_matrix = np.array([[574.73088734,   0,          170.32277285],
                                       [0,         577.94407662,  127.017184],
                                       [0        , 0        , 1   ]])

        self.distortion_coeffs = np.array([[-5.05221930e-01,  3.23562340e+00, -3.42074830e-03, 1.48988358e-04, -2.51851873e+01]])
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.bridge = CvBridge()

        # matriz fixa de remapeamento de eixos (x'=-y, y'=-x, z'=+z)
        # aplica-se tanto à translação quanto à orientação
        self.R_fix = R.from_matrix(np.array([[0, -1,  0],
                                             [-1, 0,  0],
                                             [0,  0,  1]], dtype=float))

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
        print(f"Markers detectados: {n_detected} - IDs: {ids.flatten() if ids is not None else 'Nenhum'}")

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
                    pose_msg.header.stamp = rospy.Time.now()
                    # opcional, mas recomendado: deixe claro o frame já alinhado
                    pose_msg.header.frame_id = "thermal_aligned"

                    pose_msg.pose.position.x = float(t_new[0])  # x' = -y (original)
                    pose_msg.pose.position.y = float(t_new[1])  # y' = -x (original)
                    pose_msg.pose.position.z = float(t_new[2])  # z' = +z

                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])

                    self.pose_pub.publish(pose_msg)
                    print(f"Publicado pose (ID 682): tvec_orig={tvec}, tvec_new={t_new}, quat_new={quat}")
            else:
                print("Nenhum marker ID 682 detectado neste frame.")
        else:
            print("Nenhum marker detectado.")

        # publica imagem de visualização
        republished_msg = self.bridge.cv2_to_imgmsg(image_viz, encoding='rgb8')
        self.image_pub.publish(republished_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('aruco_node')
    republisher = ImageRepublisher()
    republisher.run()

