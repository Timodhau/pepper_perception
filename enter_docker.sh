#!/bin/bash

sudo xhost +
sudo docker run --rm -it --network host -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix --name pepper_docker ros:mykinetic
