youbot_simulador
================

Interfaz para convertir los mensajes de ROS del Youbot a mensajes para el simulador V-Rep


Instrucciones
=============

Añadir el paquete youbot_simulador a la carpeta donde estén los paquetes de ROS (ros_stacks normalmente) y compilarlo con rosmake youbot-simulador (hacer antes un "rospack profile" en una terminal para actualizar la lista de paquetes)

1) Descargar V-Rep de la página oficial y descomprimirlo donde se desee (http://coppeliarobotics.com/V-REP_PRO_EDU_V3_0_5c_Linux.tar.gz)

2) Mover el archivo escena_youbot.ttt a la subcarpeta "scenes" del V-Rep

3) Mover el archivo KUKA YouBot ROS (sin UI).ttm a la subcarpeta models/robots/mobile dentro del V-Rep

4) Lanzar roscore siempre antes de abrir el V-Rep, para que el V-Rep pueda usar las funciones de ROS

5) Lanzar el nodo interface del paquete youbot-simulador (rosrun youbot_simulador interface)

6) Desde una terminal ir a la carpeta del V-Rep y lanzar el V-Rep con "./vrep scenes/escena_youbot.ttt". Se abrirá el V-Rep y cargará una escena con el Youbot ya preparado

7) Pulsar el botón de play para iniciar la simulación


En este momento podemos lanzar cualquier nodo de ROS que se comunique con el youbot, ya sean los ejemplos incluídos por Youbot para mover la base y el brazo o el paquete youbot-xsens-controller-master de Daniel y debería funcionar con el Youbot simulado.
