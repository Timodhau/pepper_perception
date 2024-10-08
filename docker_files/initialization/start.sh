#!/bin/bash

roslaunch pepper_perception pepper_perception.launch nao_ip:=$1 nao_port:=9559 network_interface:=wlo1

