#!/usr/bin/env python3
""" 
    This src is adapted from the following repo, which is under the MIT license:
    https://github.com/TIERS/ros-dwm1001-uwb-localization
"""

import rospy, time, serial, os
from dwm1001_apiCommands import DWM1001_API_COMMANDS
from geometry_msgs.msg import PoseStamped
from KalmanFilter import KalmanFilter as kf
import numpy as np
from Helpers_KF import initConstVelocityKF 

from uwb_tracking_ros.msg import CustomTag
from uwb_tracking_ros.msg import MultiTags


class dwm1001_localizer:

    def __init__(self) :
        rospy.init_node('DWM1001_Listener_Mode', anonymous=False)

        self.rate = rospy.Rate(1)
        
        self.topics = {}
        self.topics_kf = {}
        self.kalman_filters = {}
        self.tag_list_index = {}

        self.multipleTags = MultiTags()
        self.pub_tags = rospy.Publisher("/dwm1001/multiTags", MultiTags, queue_size=100) 
        self.pub_follow = rospy.Publisher(
            "/dwm1001/id_uwb_map/pose_kf",
            PoseStamped,
            queue_size=10
        )
                
        self.dwm_port = rospy.get_param('~port')
        self.verbose = rospy.get_param('~verbose', True)
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        self.serialPortDWM1001.close()
        time.sleep(1)
        self.serialPortDWM1001.open()

        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
            self.initializeDWM1001API()
            time.sleep(2)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates and process them!")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        try:
            while not rospy.is_shutdown():
                serialReadLine = self.serialPortDWM1001.read_until()
                try:
                    self.publishTagPositions(serialReadLine)    

                    serDataList = [x.strip() for x in serialReadLine.strip().split(b',')]
                    if b"POS" in serDataList[0] :
                        tag_id = int(serDataList[1])  
                        tag_macID = str(serDataList[2], 'UTF8')
                        t_pose_x = float(serDataList[3])
                        t_pose_y = float(serDataList[4])
                        t_pose_z = float(serDataList[5])   

                        t_pose_list = [t_pose_x, t_pose_y, t_pose_z]

                        # ✅ Correção: só cria t_pose_xyz se não tiver NaN
                        if(np.isnan(t_pose_list).any()):
                            continue
                        else:
                            t_pose_xyz = np.array(t_pose_list) 
                            t_pose_xyz.shape = (len(t_pose_xyz), 1)   

                        if tag_id not in self.kalman_filters:
                            A = np.zeros((6,6))
                            H = np.zeros((3, 6))  
                            self.kalman_filters[tag_id] = kf(A, H, tag_macID) 
                        
                        if self.kalman_filters[tag_id].isKalmanInitialized == False:  
                            A, B, H, Q, R, P_0, x_0  = initConstVelocityKF() 
                            self.kalman_filters[tag_id].assignSystemParameters(A, B, H, Q, R, P_0, x_0)  
                            self.kalman_filters[tag_id].isKalmanInitialized = True                            
                   
                        self.kalman_filters[tag_id].performKalmanFilter(t_pose_xyz, 0)  
                        t_pose_vel_kf = self.kalman_filters[tag_id].x_m  
                        t_pose_kf = t_pose_vel_kf[0:3]  
                        self.publishTagPoseKF(tag_id, "uwb_map", t_pose_kf)
                        
                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")

        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001.read_until()
            if b"reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001.close()


    def publishTagPositions(self, serialData):
        ser_pose_data = [x.strip() for x in serialData.strip().split(b',')]
        if b"POS" in ser_pose_data[0] :
            tag_id = str(ser_pose_data[1], 'UTF8')  
            tag_macID = str(ser_pose_data[2], 'UTF8')

            ps = PoseStamped()
            ps.pose.position.x = float(ser_pose_data[3])
            ps.pose.position.y = float(ser_pose_data[4])
            ps.pose.position.z = float(ser_pose_data[5])
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            ps.header.stamp = rospy.Time.now()   
            ps.header.frame_id = "uwb_map" 

            raw_pose_xzy = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]

            tag = CustomTag()
            tag.header = ps.header
            tag.pose_x = ps.pose.position.x
            tag.pose_y = ps.pose.position.y
            tag.pose_z = ps.pose.position.z
            tag.orientation_x = ps.pose.orientation.x
            tag.orientation_y = ps.pose.orientation.y
            tag.orientation_z = ps.pose.orientation.z
            if hasattr(tag, 'orientation_w'):
                tag.orientation_w = ps.pose.orientation.w

            if tag_id not in self.topics:
                self.topics[tag_id] = rospy.Publisher("/dwm1001/id_" + tag_macID + "/pose", PoseStamped, queue_size=10)
                self.tag_list_index[tag_id] = len(self.multipleTags.TagsList)
                self.multipleTags.TagsList.append(tag) 
            
            if(np.isnan(raw_pose_xzy).any()): 
                pass
            else:
                self.topics[tag_id].publish(ps) 
                self.multipleTags.TagsList[self.tag_list_index[tag_id]] = tag

            self.pub_tags.publish(self.multipleTags)    
                        

    # Publish Tag positions using KF 
    def publishTagPoseKF(self, id_int, id_str, kfPoseData):

        ps = PoseStamped()
        ps.pose.position.x = float(kfPoseData[0])
        ps.pose.position.y = float(kfPoseData[1])
        ps.pose.position.z = float(kfPoseData[2])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = rospy.Time.now()   
        ps.header.frame_id = id_str 

        if id_int not in self.topics_kf:
            self.topics_kf[id_int] = rospy.Publisher(
                "/dwm1001/id_" + str(id_str) + "/pose_kf",
                PoseStamped,
                queue_size=10
            )

        self.topics_kf[id_int].publish(ps)
        self.pub_follow.publish(ps)
            

    def initializeDWM1001API(self):
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)


def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
