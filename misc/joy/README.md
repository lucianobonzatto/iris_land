
# Conectando o Controle 8BitDo ao ROS 2 Jazzy com joy_node

Controle **8BitDo Ultimate 2C Wireless Controller** ao **ROS 2 Jazzy**, usando o pacote `joy`.

---

## 1. Identificar o Dispositivo de Entrada

Execute o comando abaixo para listar todos os dispositivos de entrada disponíveis:

```bash
sudo evtest
````

Procure por uma linha semelhante a:

```
/dev/input/event20: 8BitDo Ultimate 2C Wireless Controller
```

Esse será o dispositivo usado para leitura dos comandos do controle.

> /dev/input/event20

---

## 2. Executar o joy_node

Com o dispositivo identificado (ex: `/dev/input/event20`), execute:

```bash
ros2 run joy joy_node --ros-args -p dev:="/dev/input/event20"
```

Você verá a seguinte saída indicando que o dispositivo foi carregado com sucesso:

```
[INFO] [xxxx.xxxxxxx] [joy_node]: Opened joystick: 8BitDo Ultimate 2C Wireless Controller.  deadzone: 0.050000
```

---

## 3. Verificar os Dados do Controle no ROS

Abra outro terminal e execute:

```bash
ros2 topic echo /joy
```

Ao movimentar os botões e analógicos do controle, você verá mensagens no formato:

```yaml
header:
  stamp:
    sec: 123
    nanosec: 456000000
  frame_id: ''
axes: [0.0, 1.0, -1.0, ...]
buttons: [0, 1, 0, 0, ...]
```

---

## 🛠️ 4. Arquivo de Parâmetros

Crie um arquivo `joy_params.yaml` para configurar o joy_node` com mais flexibilidade

```yaml
joy_node:
  ros__parameters:
    dev: "/dev/input/event20"
    deadzone: 0.05
    autorepeat_rate: 20.0
```

Execute com:

```bash
ros2 run joy joy_node --ros-args --params-file joy_params.yaml
```

---

## 5. Permissões

Se você encontrar erros de permissão ao acessar `/dev/input/event20`, adicione seu usuário ao grupo `input`:

```bash
sudo usermod -aG input $USER
newgrp input
```
