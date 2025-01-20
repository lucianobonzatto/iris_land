#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import cv2.aruco as aruco
from math import pi, sin, cos
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

class ImageReader:
    def __init__(self):
        self._initialize_topics()
        self._initialize_aruco_settings()
        self._initialize_transform_matrices()

    def _initialize_topics(self):
        self.image_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/aruco/image', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('/aruco/pose', PoseStamped, queue_size=10)

    def _initialize_aruco_settings(self):
        self.camera_matrix = np.array(
            [
                [3.02573320e03, 0.00000000e00, 1.02641519e03],
                [0.00000000e00, 2.98476190e03, 2.69918299e02],
                [0.00000000e00, 0.00000000e00, 1.00000000e00],
            ]
        )
        self.distortion_coeffs = np.array(
            [-0.31855945, -0.04039797, 0.00156687, 0.00949025, 0.09074052]
        )

        self.marker_sizes = {272: 0.15, 682: 0.08, 0: 0.25}
        self.dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def _initialize_transform_matrices(self):
        # Cria a matriz de transformação Landpad -> Aruco

        Position_272 = np.array([-0.255, -0.160, 0])
        Rotation_272 = np.array([
            [-1, 0, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0,-1, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0, 0, 1]    # Eixo Z permanece o mesmo
        ])

        Position_682 = np.array([0.043, 0.038, 0])
        Rotation_682 = np.eye(3)

        Position_000 = np.array([0.320, 0.215, 0])
        Rotation_000 = np.array([
            [-1, 0, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0,-1, 0],  # Cos(180) = -1, Sin(180) = 0
            [ 0, 0, 1]    # Eixo Z permanece o mesmo
        ])

        self.TM_Aruco_To_Landpad_272 = np.eye(4)
        self.TM_Aruco_To_Landpad_272[:3, :3] = Rotation_272
        self.TM_Aruco_To_Landpad_272[:3, 3] = Position_272

        self.TM_Aruco_To_Landpad_682 = np.eye(4)
        self.TM_Aruco_To_Landpad_682[:3, :3] = Rotation_682
        self.TM_Aruco_To_Landpad_682[:3, 3] = Position_682

        self.TM_Aruco_To_Landpad_000 = np.eye(4)
        self.TM_Aruco_To_Landpad_000[:3, :3] = Rotation_000
        self.TM_Aruco_To_Landpad_000[:3, 3] = Position_000

        self.TM_Landpad_To_Aruco_272 = np.linalg.inv(self.TM_Aruco_To_Landpad_272)
        self.TM_Landpad_To_Aruco_682 = np.linalg.inv(self.TM_Aruco_To_Landpad_682)
        self.TM_Landpad_To_Aruco_000 = np.linalg.inv(self.TM_Aruco_To_Landpad_000)
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        pose_msg, image = self.position_detect(image)

        # image message creation
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.image_pub.publish(image_msg)

        # pose message creation
        if pose_msg is not None:
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_frame"
            self.pose_pub.publish(pose_msg)

    def position_detect(self, image):
        pose_msg = None
        gray_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray_frame, self.dictionary, parameters=self.parameters)
        return_image = aruco.drawDetectedMarkers(image, corners, ids)

        if ids is not None:
            ids_to_process = [
                (i, id[0])
                for i, id in enumerate(ids)
                if id[0] in self.marker_sizes
            ]

            positions = []
            orientations = []
            for i, marker_id in ids_to_process:
                marker_length = self.marker_sizes[marker_id]
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                    corners[i : i + 1],
                    marker_length,
                    self.camera_matrix,
                    self.distortion_coeffs,
                )

                return_image = cv2.drawFrameAxes(return_image,self.camera_matrix,self.distortion_coeffs,rvecs,tvecs,marker_length)
            
                if rvecs is not None and tvecs is not None:
                    tvecs = np.squeeze(tvecs)
                    rvecs = np.squeeze(rvecs)
                    pos_landpad_to_camera, rot_landpad_to_camera = self._landpad_to_camera(tvecs, rvecs, marker_id)

                    # Armazena os valores na lista
                    if pos_landpad_to_camera is not None and rot_landpad_to_camera is not None:
                        positions.append(pos_landpad_to_camera)
                        orientations.append(rot_landpad_to_camera)
                    
            if positions and orientations:
                avg_position = np.mean(positions, axis=0)
                avg_orientation = np.mean(orientations, axis=0)

                # Converte os ângulos médios de Euler para quaternion
                quaternion = R.from_euler('ZYX', avg_orientation, degrees=True).as_quat()

                # Preenche o pose_msg com os valores médios
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = avg_position[0]
                pose_msg.pose.position.y = avg_position[1]
                pose_msg.pose.position.z = avg_position[2]

                pose_msg.pose.orientation.x = quaternion[0]
                pose_msg.pose.orientation.y = quaternion[1]
                pose_msg.pose.orientation.z = quaternion[2]
                pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg, return_image

    def _landpad_to_camera(self, Tvec, Rvec, id):
        if id not in [272, 682, 0]:
            return None, None

        # Cria a matriz de transformação Aruco -> Câmera
        r = R.from_rotvec(Rvec)
        TM_Aruco_To_Camera = np.eye(4)
        TM_Aruco_To_Camera[:3, :3] = r.as_matrix()
        TM_Aruco_To_Camera[:3, 3] = Tvec

        # Cria a matriz de transformação Landpad -> Câmera
        TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_000
        if id == 272:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_272
        elif id == 682:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_682
        elif id == 0:
            TM_Landpad_To_Camera = TM_Aruco_To_Camera @ self.TM_Landpad_To_Aruco_000

        pos_landpad_to_camera = TM_Landpad_To_Camera[:3, 3]
        pos_landpad_to_camera[1] = -pos_landpad_to_camera[1]

        rotation_landpad_to_camera = R.from_matrix(TM_Landpad_To_Camera[:3, :3])
        rot_landpad_to_camera = rotation_landpad_to_camera.as_euler('ZYX', degrees=True) # roll, pitch, yaw

        return pos_landpad_to_camera, rot_landpad_to_camera

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    print(cv2.__version__)

    rospy.init_node('aruco_node')

    republisher = ImageReader()
    republisher.run()
