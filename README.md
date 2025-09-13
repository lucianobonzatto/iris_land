
# iris_land

## run simulation

1. terminal 1
```
$ roscore
```


2. terminal 2
```
$ roscd iris_land/
$ ../shell_script/
$ ./iris_sim.sh
```

3. ativa uma das opções na tela de joy e aciona a publicação

4. terminal 3
```
$ rosrun iris_land iris_mng
```

5. terminal 4
```
$ rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
```

## iris simulation

editar o arquivo em 

```
src/Firmware/Tools/sitl_run.sh

src/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf

src/Firmware/Tools/sitl_gazebo/models/fpv_cam/fpv_cam.sdf
```

https://github.com/piradata/wpg

https://github.com/piradata/PX4-Autopilot


### install

```
mkdir -p ~/src/
cd ~/src/
git clone https://github.com/piradata/PX4-Autopilot.git Firmware
cd Firmware/
make px4_sitl gazebo_iris
```

### run
```
cd ~/src/Firmware/
make px4_sitl gazebo_iris
roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'
```

## aruco node

rosrun iris_land aruco_node --performance (roda so com o topico de pose)
rosrun iris_land aruco_node (publica as imagens tb, menos FPS)


se for otimizar algo, mexe somente no struct config que tem no .cpp, recomendo
mexer somente no jointmaxpnpiterations


## radio connection


## Identificar o Dispositivo de Entrada

Execute o comando abaixo para listar todos os dispositivos de entrada disponíveis:

```bash
sudo evtest
````

Procure por uma linha semelhante a:

```
/dev/input/event18:	OpenTX FrSky Taranis Joystick
```

Esse será o dispositivo usado para leitura dos comandos do controle.

> /dev/input/event18

---

## 2. Executar o joy_node

Com o dispositivo identificado (ex: `/dev/input/event18`), execute:

```bash
ros2 run joy joy_node --ros-args -p dev:=/dev/input/event18
```

Você verá a seguinte saída indicando que o dispositivo foi carregado com sucesso:

```
[INFO] [xxxx.xxxxxxx] [joy_node]: Opened joystick: 8BitDo Ultimate 2C Wireless Controller.  deadzone: 0.050000
```

---


## Arquivo de Parâmetros

Crie um arquivo `joy_params.yaml` para configurar o joy_node` com mais flexibilidade

```yaml
joy_node:
  ros__parameters:
    dev: "/dev/input/event18"
    deadzone: 0.05
    autorepeat_rate: 20.0
```

Execute com:

```bash
ros2 run joy joy_node --ros-args --params-file /home/lukn23/ros2_ws/src/iris_land/misc/joy/joy_params
```


yaml