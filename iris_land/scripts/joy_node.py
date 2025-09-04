#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn, OverrideRCIn
import tkinter as tk
import threading

class RCOverrideGUI(Node):
    def __init__(self):
        super().__init__('rc_override_gui_publisher')

        self.publish_active = False

        # Publisher
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # Sliders e variável do canal 7
        self.throttle = 1500
        self.yaw = 1500
        self.pitch = 1500
        self.roll = 1500
        self.channel_7_val = 0

        # Cria a interface gráfica em um thread
        gui_thread = threading.Thread(target=self.create_gui)
        gui_thread.start()

        # Loop de publicação
        self.timer = self.create_timer(0.1, self.publish_callback)  # 10 Hz

    def create_gui(self):
        root = tk.Tk()
        root.title("Controle RC Override")

        # Throttle
        tk.Label(root, text="Throttle").pack()
        self.throttle_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
        self.throttle_slider.set(1500)
        self.throttle_slider.pack()

        # Yaw
        tk.Label(root, text="Yaw").pack()
        self.yaw_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
        self.yaw_slider.set(1500)
        self.yaw_slider.pack()

        # Pitch
        tk.Label(root, text="Pitch").pack()
        self.pitch_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
        self.pitch_slider.set(1500)
        self.pitch_slider.pack()

        # Roll
        tk.Label(root, text="Roll").pack()
        self.roll_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
        self.roll_slider.set(1500)
        self.roll_slider.pack()

        # Canal 7
        self.channel_7 = tk.IntVar()
        self.channel_7.set(0)
        tk.Radiobutton(root, text="p1 - stop", variable=self.channel_7, value=982).pack()
        tk.Radiobutton(root, text="p2 - land", variable=self.channel_7, value=1494).pack()
        tk.Radiobutton(root, text="p3 - follow", variable=self.channel_7, value=2006).pack()

        # Botão de alternar publicação
        self.toggle_button = tk.Button(root, text="Iniciar Publicação", command=self.toggle_publish)
        self.toggle_button.pack()

        root.mainloop()

    def toggle_publish(self):
        self.publish_active = not self.publish_active
        if self.publish_active:
            self.toggle_button.config(text="Parar Publicação")
        else:
            self.toggle_button.config(text="Iniciar Publicação")
    
    def publish_callback(self):
        if self.publish_active:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [
                self.throttle_slider.get(),
                self.yaw_slider.get(),
                self.pitch_slider.get(),
                self.roll_slider.get(),
                0, 0,
                self.channel_7.get(),
                0,0,0,0,0,0,0,0,0,0,0
            ]
            self.rc_pub.publish(rc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RCOverrideGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
