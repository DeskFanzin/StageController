# StageController
Um nodo que faz o robô diferencial do Stage ir até dois objetivos (coordenadas (x = 5, y = 4) e
(x = 4, y = −4) no mapa, respectivamente) utilizando ROS2 Humble.

Como rodar?

Primeiramente, baixe o workspace e compile utilizando o comando:

```
colcon build
```

Depois, abra dois terminais. No primeiro, rode o launcher do Stage.

```
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
```

No segundo, rode o nodo em si.

```
ros2 run my_py_pkg py_node
```

E observe o robô desviar dos obstáculos e chegar nos dois destinos.

Atenção: As coordenadas do mapa para as coordenadas do código podem estar diferentes, por causa que no código utilizamos a posição x e y do robô como referência.
