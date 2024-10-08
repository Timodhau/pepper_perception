!/bin/bash

(roslaunch pepper_bringup pepper_full.launch nao_ip:=$1 roscore_ip:=localhost network_interface:=$2) &
sleep 5
(roslaunch nao_interaction_launchers nao_audio_interface.launch nao_ip:=$1 nao_port:=9559 network_interface:=$2) &
sleep 3
(python /home/initialization/plugins_pepper_ros.py --ip $1)
