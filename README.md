# Stage_Controller package

## Autor: Matheus Gonçalves (157142)

Esse pacote contem um arquivo launch que inicia o simulador 2D Stage e o nodo controller_PD para fazer o robô chegar no alvo desejado.
______

### Pré-requisitos

- [git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
______

### scripts/diferencial_PD.py

Foi implementado uma classe que, a partir da leitura do laser e da posição do robô no mundo, se movimenta em direção ao alvo solicitado, desviando de obstáculos se estiverem no seu caminho.
- Para a movimentação foi utilizado um Controle Proporcional Derivativo para a posição do veículo, onde as constantes kp e kd podem ser alteradas nos parâmetros do launcher
- Para o desvio de obstáculos se dividiu os feiches do scanner pela metade centralizado-o a partir do centro do robô, se obtendo assim um cone centralizado. Foi implementado a estratégia de rotacionar na direção oposta à que tiver a menor leitura do scan.

Informações do robô:
- A posição do robô é atualizada a partir da leitura do tópico passado via parâmetro para topic_odom;
- A leitura do sensor é atualizada a partir da leitura do tópico passado via parâmetro para topic_lidar;
- A movimentação do robô 


### Vídeo

- https://youtu.be/m200MFU1CQM

### Como rodar

1. Crie um workspace com o diretório src:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
```
  
2. Clone o pacote dentro da pasta workspace/src

```
$ git clone git@github.com:mgmateus/controller_PD.git
$ cd ~/catkin_ws
```

3. Compile o pacote

```
$ catkin build
```

3.1. Se estiver usando bash

```
$ source devel/setup.bash
```

3.2. Se estiver usando zsh

```
$ source devel/setup.zsh
```

4. Inicie o launcher

```
$ roslaunch controller_PD launcher.launch 
```

5. Para modificar as posições alvo, alterar os valores para target_x e target_y

```
$ roslaunch controller_PD launcher.launch target_x:=-1.0 target_y:=2.0
```

