## _AI_LIDAR_FUSION_

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

# general description

## Introdução
Este projeto enquadra-se no contexto de um barco autónomo, desenvolvido pelo núcleo Técnico Solar Boat. Assume-se a existencia um módulo de AI que atua nas imagens provenientes das 4 cameras do barco e reconhece a presença de obstáculos. O módulo de AI publica, usando o ROS, informação relativa ao pixel onde o objeto foi encontrado. Assume-se também a existencia de um módulo que publica a informação proveniente de um sensor de distancias, por exemplo um lidar.
Este módulo foca-se em permitir a correspondencia da informação proveniente da AI detetar um objeto com os dados fornecidos pelo lidar.

## Implementação
Sabendo o FOV da camera e a resolução da frame, é possível calcular os ângulos que o pixel representativo do objeto detetado na AI faz com o eixo da camera na horizontal e na vertical, como representado nas imagens abaixo.

 <img src="https://github.com/rayvo50/images/blob/main/ai_lidar_fusion/FOVH.png?raw=true" width="400">   <img src="https://github.com/rayvo50/images/blob/main/ai_lidar_fusion/FOVV.png?raw=true" width="400">

 <img src="https://github.com/rayvo50/images/blob/main/ai_lidar_fusion/AngH.png?raw=true" width="400">   <img src="https://github.com/rayvo50/images/blob/main/ai_lidar_fusion/AngV.png?raw=true" width="400">
 
Tendo obtido os angulos referentes ao pixel fornecido, é necessário filtrar a nuvem de pontos de modo a obter-se apenas os pontos que estejam nos angulos corretos. Estando a nuvem de pontos representada num referencial XYZ, basta percorrer o vetor dos pontos, calculando os angulos que cada ponto faz com a horizontal e a vertical, e guardar apenas aqueles em que os angulos estejam perto dos angulos calculados anteriormente. A figura abaixo ilustra como se calcularia o angulo na horizontal de um pixel detetado na camera frontal.

 <img src="https://github.com/rayvo50/images/blob/main/ai_lidar_fusion/esquema_angulos.png?raw=true" width="400">

Assim, obtem-se um conjunto restrito de pontos, sendo possivel obter as coordenadas aproximadas do objeto detetado pela AI.
 
## Correr o programa
Antes de lançar o ros como habitualmente, é necessário alterar o ficheiro "~/catkin_ws/src/ros_adapter/scripts/server.py" para que as mensagens publicadas pelo simulador possam ser sincronizadas com o nodo de teste disponibilizado. Substituir
```py
header.stamp = rospy.Time.from_sec(request.timeInSeconds)
```
por
```py
header.stamp = rospy.Time.now()
```
Depois, a plataforma pode ser lançada como habitualmente:
```sh
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/src/ros_scenario/launch/launch_njord_stack.launch
```




