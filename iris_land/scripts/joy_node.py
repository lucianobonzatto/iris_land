import rospy
from mavros_msgs.msg import RCIn
import tkinter as tk

# Variável global para sinalizar quando enviar mensagens
publish_active = False

def publish_loop():
    rate = rospy.Rate(10)  # Define a frequência de 10 Hz
    while not rospy.is_shutdown():
        if publish_active:
            rc_msg = RCIn()
            rc_msg.header.stamp = rospy.Time.now()
            rc_msg.channels = [
                throttle_slider.get(),
                yaw_slider.get(),
                pitch_slider.get(),
                roll_slider.get(),
                0, 0,
                channel_7.get(),
                0,0,0,0,0,0,0,0,0,0,0
            ]
            rc_pub.publish(rc_msg)
            # rospy.loginfo(f"RCIn mensagem publicada: {rc_msg.channels}")
        rate.sleep()

# Função para ativar/desativar a publicação
def toggle_publish():
    global publish_active
    publish_active = not publish_active  # Alterna entre publicar e parar
    if publish_active:
        toggle_button.config(text="Parar Publicação")
    else:
        toggle_button.config(text="Iniciar Publicação")

# Função para criar a interface gráfica
def create_gui():
    # Criação da janela principal
    root = tk.Tk()
    root.title("Controle RC Override")

    # Descrições dos sliders
    tk.Label(root, text="Throttle").pack()
    global throttle_slider
    throttle_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
    throttle_slider.set(1500)
    throttle_slider.pack()

    tk.Label(root, text="Yaw").pack()
    global yaw_slider
    yaw_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
    yaw_slider.set(1500)
    yaw_slider.pack()

    tk.Label(root, text="Pitch").pack()
    global pitch_slider
    pitch_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
    pitch_slider.set(1500)
    pitch_slider.pack()

    tk.Label(root, text="Roll").pack()
    global roll_slider
    roll_slider = tk.Scale(root, from_=1000, to=2000, orient=tk.HORIZONTAL)
    roll_slider.set(1500)
    roll_slider.pack()

    # Chave de 2 posições para o canal 7
    global channel_7
    channel_7 = tk.IntVar()
    channel_7.set(0)  # Valor inicial neutro
    tk.Radiobutton(root, text="000", variable=channel_7, value=0).pack()
    tk.Radiobutton(root, text="100", variable=channel_7, value=100).pack()
    tk.Radiobutton(root, text="200", variable=channel_7, value=200).pack()
    tk.Radiobutton(root, text="300", variable=channel_7, value=300).pack()
    tk.Radiobutton(root, text="400", variable=channel_7, value=400).pack()

    # Botão para alternar a publicação
    global toggle_button
    toggle_button = tk.Button(root, text="Iniciar Publicação", command=toggle_publish)
    toggle_button.pack()

    # Loop principal da interface gráfica
    root.mainloop()

if __name__ == '__main__':
    try:
        # Inicializa o nó ROS
        rospy.init_node('rc_override_gui_publisher', anonymous=True)
        
        # Cria o publisher
        rc_pub = rospy.Publisher('/rc/in', RCIn, queue_size=10)

        # Cria a interface gráfica em um thread separado
        import threading
        gui_thread = threading.Thread(target=create_gui)
        gui_thread.start()

        # Inicia o loop de publicação
        publish_loop()

    except rospy.ROSInterruptException:
        pass
