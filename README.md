## AI_Lidar_fusion

<img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/poweredby.png?raw=true" width="150">

## Introdução
Este projeto foi desenvolvido no âmbito do recrutamento para o núcleo Técnico Solar Boat. Assume-se a existencia um módulo de AI que atua nas imagens provenientes das 4 cameras do barco e reconhece a presença de obstáculos. O módulo de AI publica, usando o ROS, informação relativa ao pixel onde o objeto foi encontrado. Assume-se também a existencia de um módulo que publica a informação proveniente de um sensor de distâncias, por exemplo um lidar.
Este módulo foca-se em permitir a correspondencia da informação proveniente da AI detetar um objeto com os dados fornecidos pelo lidar.

## Implementação
Sabendo o FOV da camera e a resolução da frame, é possível calcular os ângulos que um pixel (representativo do objeto detetado na AI) faz com o eixo da camera na horizontal e na vertical, como representado nas imagens abaixo.

 <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/FOVH.png?raw=true" width="400">   <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/FOVV.png?raw=true" width="400">

 <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/AngH.png?raw=true" width="400">   <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/AngV.png?raw=true" width="400">
 
Tendo obtido os angulos referentes ao pixel fornecido, é necessário filtrar a nuvem de pontos de modo a obter-se apenas os pontos que estejam nos angulos corretos. Estando a nuvem de pontos representada num referencial XYZ, basta percorrer o vetor dos pontos, calculando os angulos que cada ponto faz com a horizontal e a vertical, e guardar apenas aqueles em que os angulos estejam perto dos angulos calculados anteriormente. A figura abaixo ilustra como se calcularia o angulo na horizontal de um pixel detetado na camera frontal.

 <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/esquema_angulos.png?raw=true" width="400">

Assim, obtem-se um conjunto restrito de pontos, sendo possivel obter as coordenadas aproximadas do objeto detetado pela AI.


## Funções

Descrição das funções mais importantes do projeto

* `calc_angle`:
   Recorrendo às imagens acima, bem como à imagem abaixo, facilmente se percebe o calculo feito nesta função.
   Primeiro calcula-se "dipH" usando "FOVx" e "w/2" e depois "angleH" a partir de "x" e "dipH". Para o angulo vertical o processo é identico.
   
   <img src="https://github.com/rayvo50/ros-ai_lidar_fusion/blob/main/images/calc_angle.png?raw=true" width="300">

* `getpointsub`:
  A partir de "angleH", "angleV" e "desvio" definem-se os limites angulares em que os pontos serão filtrados.
  O calculo dos angulos é diferente conforme qual for a camera, uma vez que a sua orientação é diferente. Na Figura acima está esquematizado como se pode proceder para calcular os angulos. No caso dos pares front-back e port-starboard o calculo daria o mesmo resultado. Assim, é necessário retirar os pontos que estão no lado oposto (aka na analise da camere da frente remover os pontos com y negtivo). Para filtrar os pontos basta calcular para cada ponto os angulos e guardá-lo apenas se estiver dentro dos limites.
  
* `calc_distance`:
Esta função apenas calcula a média dos pontos recebidos por `getpointsub`. Numa implementação futura, esta função pode ser melhorada de forma a publicar os dados para o controlador de direção do barco.

* Sobre o `ApproximateTimeSynchronizer()`:
 Este filtro é necessário para garantir a sincronização das mensagens publicadas pelo simulador e pelo nodo de teste. O filro recebe as mensagens dos tópicos e retorna-as se encontrar mensagens com uma "time stamp" semelhante. Isto permite que a imagem analizada pela AI terá o mesmo instante de tempo que a nuvem de pontos do lidar. É tambem necessário para permitir a análise de dados provenientes de vários tópicos dentro da mesma função de callback.

## Construir o nodo
_Nota: Não foi possível testar estas instruções pelo que se surgirem problemas é só contactar-me._

Começar por criar um novo package dentro do workspace 
```sh
cd ~/catkin_ws
catkin_create_pkg ai_lidar_fusion rospy std_msgs sensor_msgs
```
Depois é necessário copiar os ficheiros deste repositório e troná-los executaveis com o comando:
```sh
chmod +x <path>/converter.py
chmod +x <path>/test_pub.py
chmod +x <path>/camera.py
```
Como o nodo de teste usa uma menssagem customizada, é possível que seja necessáro fazer alterações em alguns ficheiros. Mais informação contactar-me ou consultar http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv. 

Finalmente correr
```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```


## Correr o programa
Para que os pontos do lidar possam ser visualizados no rviz sem estarem espelhados, é necessário fazer uma pequena alteração no simulador. No menu do lado esquerdo selecionar NjordVessel > SensorRig > Lidar. No menu que aparece do lado direito, alterar o campo "Scale X" para "-1".
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
Para correr o nodo:
```sh
rosrun ai_lidar_fusion converter.py
```
Para testar com um certo pixel fazer uso do ficheiro sub_test.py, alterando-o para testar vários pixeis.
```sh
rosrun ai_lidar_fusion test_pub.py
```
Para facilitar a escolha do pixel a ser testado, fazer uso do ficheiro camera.py. Passando o rato sobre a imagem é possível ver qual o pixel que se pretende.
```sh
rosrun ai_lidar_fusion camera.py
```



