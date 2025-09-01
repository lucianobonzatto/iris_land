
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


