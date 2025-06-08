import os
import rospy
import rospkg 
import datetime
import subprocess
import tkinter as tk
from tkinter import Label, Entry, Button, Radiobutton, StringVar, Scale
from iris_land.msg import controllers_gain

class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Configuração de Controladores")
        self.root.resizable(False, False)
        
        self.entries = {}
        self.par_pub = rospy.Publisher('/PID/parameters', controllers_gain, queue_size=10)
        self.gains = controllers_gain()

        self.package_path = rospkg.RosPack().get_path("iris_land")
        self.gains_file = self.package_path + "/config/gains.txt"
        self.bag_path = self.package_path + "/../bag/"

        if not os.path.exists(self.bag_path):
            os.mkdir(self.bag_path)

        self.create_widgets()
        self.load_gains()
        
    def create_widgets(self):

        self.action_frame = tk.Frame(self.root)
        self.action_frame.grid(row=1, column=0, columnspan=1, pady=5)

        self.submit_button = Button(self.action_frame, text="Aplicar", command=self.apply_gains)
        self.submit_button.grid(row=0, column=0, padx=5)

        self.save_button = Button(self.action_frame, text="Salvar", command=self.save_gains)
        self.save_button.grid(row=0, column=1, padx=5)

        # Frame para os ganhos PID
        self.pid_frame = tk.LabelFrame(self.root, text="Ganhos PID")
        self.pid_frame.grid(row=0, column=0, padx=10, pady=10)

        self.create_gain_entries("PID", ["P", "I", "D"], 0, 0, frame=self.pid_frame)

        # Frame para parâmetros gerais
        self.param_frame = tk.LabelFrame(self.root, text="Parâmetros Gerais")
        self.param_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # Labels e Entradas de Velocidade
        Label(self.param_frame, text="Linear Vel").grid(row=0, column=0, padx=5, pady=2)
        self.linear_vel = Entry(self.param_frame, width=7)
        self.linear_vel.insert(0, "1.0")
        self.linear_vel.grid(row=1, column=0, padx=5, pady=2)

        Label(self.param_frame, text="Angular Vel").grid(row=0, column=1, padx=5, pady=2)
        self.angular_vel = Entry(self.param_frame, width=7)
        self.angular_vel.insert(0, "1.0")
        self.angular_vel.grid(row=1, column=1, padx=5, pady=2)

        # Altitude (label + scale)
        Label(self.param_frame, text="Altitude").grid(row=2, column=0, columnspan=2, pady=2)
        self.scale = Scale(self.param_frame, from_=1.0, to=5.0, resolution=0.01,
                        orient="horizontal", length=200)
        self.scale.set(2.0)
        self.scale.grid(row=3, column=0, columnspan=2, padx=5, pady=2)

        # Frame para botões do bag
        self.bag_frame = tk.LabelFrame(self.root, text="bag")
        self.bag_frame.grid(row=0, column=2, columnspan=2, padx=10, pady=10)

        self.start_bag_button = Button(self.bag_frame, text="Start Bag", command=self.start_bag)
        self.start_bag_button.grid(row=1, column=0, padx=5)

        self.stop_bag_button = Button(self.bag_frame, text="Stop Bag", command=self.stop_bag)
        self.stop_bag_button.grid(row=1, column=1, padx=5)

        Label(self.bag_frame, text="Nome do arquivo:").grid(row=2, column=0, columnspan=2)
        self.entry_text = Entry(self.bag_frame, width=20)
        self.entry_text.grid(row=3, column=0, columnspan=2, pady=5)


    def create_gain_entries(self, controller_type, gains_Text, row_start, column_start, frame=None):
        if frame is None:
            frame = self.root

        dimensions = ["x", "y", "z", "yaw"]

        for i, gain in enumerate(gains_Text):
            Label(frame, text=gain).grid(row=row_start + i + 1, column=column_start, sticky="e")

        for j, dimension in enumerate(dimensions):
            Label(frame, text=dimension).grid(row=row_start, column=column_start + j + 1)

            for i, gain in enumerate(gains_Text):
                entry = Entry(frame, width=5)
                entry.insert(0, "0.0")
                entry.grid(row=row_start + i + 1, column=column_start + j + 1, padx=5, pady=5)
                self.entries[f"{controller_type}_{gain}_{dimension}"] = entry


    def save_gains(self):
        
        print(self.gains_file)

        with open(self.gains_file, 'w') as file:
            for key, entry in self.entries.items():
                value = entry.get()
                file.write(f"{key}: {value}\n")

    def start_bag(self):
        rosbag_command = f"rosnode kill rosbag_node"
        process = subprocess.Popen(rosbag_command, shell=True)
        process.wait()

        bag_filename = self.bag_path + "/" + self.entry_text.get()
        rosbag_command = f"rosbag record -o {bag_filename} -a __name:=rosbag_node"

        subprocess.Popen(rosbag_command, shell=True)

    def stop_bag(self):
        rosbag_command = f"rosnode kill rosbag_node"
        process = subprocess.Popen(rosbag_command, shell=True)
        process.wait()

    def load_gains(self):
        try:
            with open(self.gains_file, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    key, value = line.strip().split(":")
                    self.entries[key].delete(0, "end")
                    self.entries[key].insert(0, value.strip())
        except FileNotFoundError:
            print("Gains file not found. Using default values.")
            pass

    def apply_gains(self):
        self.gains.pid_ctrl.x.p_gain   = float(self.get_entry("PID", "P", "x"  ).get())
        self.gains.pid_ctrl.x.i_gain   = float(self.get_entry("PID", "I", "x"  ).get())
        self.gains.pid_ctrl.x.d_gain   = float(self.get_entry("PID", "D", "x"  ).get())
        self.gains.pid_ctrl.y.p_gain   = float(self.get_entry("PID", "P", "y"  ).get())
        self.gains.pid_ctrl.y.i_gain   = float(self.get_entry("PID", "I", "y"  ).get())
        self.gains.pid_ctrl.y.d_gain   = float(self.get_entry("PID", "D", "y"  ).get())
        self.gains.pid_ctrl.z.p_gain   = float(self.get_entry("PID", "P", "z"  ).get())
        self.gains.pid_ctrl.z.i_gain   = float(self.get_entry("PID", "I", "z"  ).get())
        self.gains.pid_ctrl.z.d_gain   = float(self.get_entry("PID", "D", "z"  ).get())
        self.gains.pid_ctrl.yaw.p_gain = float(self.get_entry("PID", "P", "yaw").get())
        self.gains.pid_ctrl.yaw.i_gain = float(self.get_entry("PID", "I", "yaw").get())
        self.gains.pid_ctrl.yaw.d_gain = float(self.get_entry("PID", "D", "yaw").get())

        self.gains.altitude = self.scale.get()

        self.gains.linear_vel = float(self.linear_vel.get())
        self.gains.angular_vel = float(self.angular_vel.get())

        self.par_pub.publish(self.gains)

    def get_entry(self, controller_type, gain, dimension):
        return self.entries[f"{controller_type}_{gain}_{dimension}"]

def main():
    teste = controllers_gain()

    rospy.init_node('controller_gui', anonymous=True)
    root = tk.Tk()
    app = ControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
