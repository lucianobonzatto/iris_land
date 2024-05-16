import os
import csv
import glob
import rospy
import rosbag
import rospkg
import datetime
import subprocess
import pandas as pd
import tkinter as tk
from tkinter import Label, Entry, Button, Radiobutton, StringVar
from iris_land.msg import controllers_gain
from tf.transformations import euler_from_quaternion
import genpy

class BagReader:
    def __init__(self):
        print("===")
        originals_folder = os.path.join("/home/lukn23/bag/tello/PID/original")
        data = glob.glob(originals_folder + "/*.bag")
        print(originals_folder, ": ", data.__len__(), " bags")
        self.time_list = []

        self.output_folder = os.path.join("/home/lukn23/tello_ws/src/iris_land/log/csv/PID")

        for bag_name in data:
            try:
                print(bag_name)
                with rosbag.Bag(bag_name, "r") as bag:
                    value = {
                        'start_time': bag.get_start_time(),
                        'end_time': bag.get_end_time(),
                        'name': bag_name.replace(originals_folder+"/", "").replace(".bag", "")
                    }
                    self.time_list.append(value)
                    bag.close()

            except rosbag.ROSBagException as e:
                rospy.logerr("Erro ao reproduzir o arquivo de bag: %s", str(e))
        print("===")

    def save_to_csv(self, data, filename):
        data = pd.DataFrame(data)
        if not data.empty:
            data.to_csv(filename, index=False)

    def read_bag(self):
        # bag_filename = "/home/lukn23/bag/tello/PID/line_rot.bag"
        bag_filename = "/home/lukn23/bag/tello/PID/move_z.bag"

        try:
            print(bag_filename, ": ", self.time_list.__len__())
            with rosbag.Bag(bag_filename, "r") as bag:
                for time in self.time_list:
                    folder_name = self.output_folder + "/" + time['name']
                    if not os.path.exists(folder_name):
                        os.makedirs(folder_name)

                    print(folder_name)

                    start_time_ros = genpy.Time.from_sec(time['start_time'])
                    end_time_ros = genpy.Time.from_sec(time['end_time'])
                    self.tello_odom = []
                    self.aruco_pose = []
                    self.magni_pose = []

                    for topic, msg, t in bag.read_messages(start_time=start_time_ros, end_time=end_time_ros):
                        if rospy.is_shutdown():
                            bag.close()
                            return -1
                        
                        print(t.to_sec(), end="\r")
                        self.topic_tratment(topic, msg, t)
                    print("")
                    self.save_to_csv(self.tello_odom, folder_name + "/tello_odom.csv")
                    self.save_to_csv(self.aruco_pose, folder_name + "/aruco_pose.csv")
                    self.save_to_csv(self.magni_pose, folder_name + "/magni_pose.csv")

                bag.close()

        except rosbag.ROSBagException as e:
            rospy.logerr("Erro ao reproduzir o arquivo de bag: %s", str(e))

    def topic_tratment(self, topic, msg, t):
        if topic == "/tello/odom":
            data = {
                'timestamp': t.to_sec(),
                'X_vel':msg.twist.twist.linear.x,
                'Y_vel':msg.twist.twist.linear.y,
                'Z_vel':msg.twist.twist.linear.z,
                'R_vel':msg.twist.twist.angular.z,
                'X_pose': msg.pose.pose.position.x,
                'Y_pose': msg.pose.pose.position.y,
                'Z_pose': msg.pose.pose.position.z,
            }

            self.tello_odom.append(data)
        elif topic == "/aruco/pose":
            data = {
                'timestamp': t.to_sec(),
                'X_pose': msg.pose.position.x,
                'Y_pose': msg.pose.position.y,
                'Z_pose': msg.pose.position.z,
                'R_pose': msg.pose.orientation.x,
            }

            self.aruco_pose.append(data)
        elif topic == "odom":
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            # print(msg.pose.pose)

            data = {
                'timestamp': t.to_sec(),
                'X_pose': msg.pose.pose.position.x,
                'Y_pose': msg.pose.pose.position.y,
                'Z_pose': msg.pose.pose.position.z,
                'R_pose': yaw
            }

            self.magni_pose.append(data)

            

def main():
    rospy.init_node("read_bag_node", anonymous=True)
    app = BagReader()
    app.read_bag()


if __name__ == "__main__":
    main()
