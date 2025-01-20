import cv2
import numpy as np

# Carregar a imagem do marcador
marker_image = cv2.imread('iris_land/scripts/resize.png')

# Verificar se a imagem foi carregada corretamente
if marker_image is None:
    print("Erro ao carregar a imagem. Verifique o caminho do arquivo.")
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
img = np.zeros((240, 320, 3), dtype=np.uint8)

# Adicionar o evento de rolagem do mouse e clique do mouse
cv2.namedWindow("Image")
cv2.setMouseCallback("Image", mouse_callback)

# Criar a janela de controle deslizante (scrollbar) para o tamanho do marcador
cv2.createTrackbar('Marker Size (%)', 'Image', 10, 100, update_marker_size)

# Inicializar os percentuais (100% de tamanho original)
marker_width_percent = 10
marker_height_percent = 10

while True:
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

    # Exibir a imagem com o marcador ajustado
    cv2.imshow("Image", img_copy)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # Pressione 'q' para sair
        break

cv2.destroyAllWindows()
