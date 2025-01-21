import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Inicializar o nó ROS
rospy.init_node('marker_publisher', anonymous=True)
image_pub = rospy.Publisher('/iris/usb_cam/image_raw', Image, queue_size=10)
bridge = CvBridge()

# Carregar a imagem do marcador
# marker_image = cv2.imread('resize.png')
marker_image = cv2.imread('aruco-0.png')

camera_matrix = np.array(
    [
        [277.191356, 0.        , 320.5],
        [0.        , 277.191356, 240.5],
        [0.        , 0.        , 1.   ],
    ]
)
distortion_coeffs = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0]
)

# Verificar se a imagem foi carregada corretamente
if marker_image is None:
    rospy.logerr("Erro ao carregar a imagem. Verifique o caminho do arquivo.")
    exit()

# Definir o tamanho original do marcador
original_marker_width = marker_image.shape[1]
original_marker_height = marker_image.shape[0]

# Variáveis para armazenar a posição do marcador
marker_x, marker_y = 160, 120  # Posição centralizada para 320x240

# Função de callback para eventos do mouse
def mouse_callback(event, x, y, flags, param):
    global marker_x, marker_y
    if event == cv2.EVENT_LBUTTONDOWN:  # Quando o botão esquerdo do mouse é pressionado
        marker_x, marker_y = x, y  # Atualiza a posição do marcador

# Função de callback do scroll bar
def update_marker_size(val):
    global marker_width_percent, marker_height_percent
    # Atualiza os percentuais para largura e altura
    marker_width_percent = val
    marker_height_percent = val

# Criar uma imagem de fundo com as dimensões 320x240
img = np.ones((240, 320, 3), dtype=np.uint8) * 255

# Adicionar o evento de clique do mouse
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", mouse_callback)

# Criar a janela de controle deslizante (scrollbar) para o tamanho do marcador
cv2.createTrackbar('Marker Size (%)', 'Image', 10, 100, update_marker_size)

# Inicializar os percentuais (10% de tamanho original)
marker_width_percent = 10
marker_height_percent = 10

rate = rospy.Rate(10)  # Publicar a imagem a 10 Hz

while not rospy.is_shutdown():
    # Criar uma cópia da imagem para desenhar o marcador
    img_copy = img.copy()

    # Calcular os novos tamanhos do marcador com base no valor percentual
    new_marker_width = int(original_marker_width * marker_width_percent / 100)
    new_marker_height = int(original_marker_height * marker_height_percent / 100)

    # Redimensionar o marcador com o novo tamanho
    resized_marker = cv2.resize(marker_image, (new_marker_width, new_marker_height))

    # Definir a posição do marcador
    marker_top_left = (marker_x - new_marker_width // 2, marker_y - new_marker_height // 2)
    marker_bottom_right = (marker_x + new_marker_width // 2, marker_y + new_marker_height // 2)

    # Garantir que o marcador caiba na imagem, ajustando o ponto central
    if marker_top_left[0] < 0:
        marker_top_left = (0, marker_top_left[1])
    if marker_top_left[1] < 0:
        marker_top_left = (marker_top_left[0], 0)

    if marker_bottom_right[0] > img.shape[1]:
        marker_top_left = (img.shape[1] - new_marker_width, marker_top_left[1])
    if marker_bottom_right[1] > img.shape[0]:
        marker_top_left = (marker_top_left[0], img.shape[0] - new_marker_height)

    marker_bottom_right = (marker_top_left[0] + new_marker_width, marker_top_left[1] + new_marker_height)

    # Calcular o tamanho da região de destino na imagem
    marker_width = marker_bottom_right[0] - marker_top_left[0]
    marker_height = marker_bottom_right[1] - marker_top_left[1]

    # Verificar se a região de destino na imagem tem o tamanho adequado
    if marker_width > 0 and marker_height > 0:
        # Inserir o marcador redimensionado na região correspondente da imagem de fundo
        img_copy[marker_top_left[1]:marker_top_left[1]+marker_height, marker_top_left[0]:marker_top_left[0]+marker_width] = resized_marker[0:marker_height, 0:marker_width]

    h, w = img_copy.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1)
    img_copy = cv2.remap(img_copy, mapx, mapy, interpolation=cv2.INTER_LINEAR)

    # Publicar a imagem como mensagem ROS
    ros_image = bridge.cv2_to_imgmsg(img_copy, encoding="bgr8")
    image_pub.publish(ros_image)

    # Exibir a imagem com o marcador ajustado
    cv2.imshow("Image", img_copy)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # Pressione 'q' para sair
        break

    rate.sleep()

cv2.destroyAllWindows()
