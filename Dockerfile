FROM ros:kinetic
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    echo ros-pepper-meshes ros-pepper-meshes/accepted-ros-pepper-meshes boolean true | debconf-set-selections
RUN apt-get update -y && apt-get install -y --no-install-recommends apt-utils && apt install vim -y && apt install python-pip -y
RUN pip install --upgrade pip==20.0.2 && pip install Pillow==2.9.0
RUN apt-get update && apt-get install net-tools ros-kinetic-pepper-bringup ros-kinetic-pepper-robot ros-kinetic-naoqi-driver ros-kinetic-tf2-sensor-msgs ros-kinetic-pepper-meshes -y
# ros-kinetic-naoqi-driver

COPY ./ros_entrypoint.sh u/
COPY boot_config.json /opt/ros/kinetic/share/naoqi_driver/share/
COPY docker_files /home/

ENV PYTHONPATH "${PYTHONPATH}:/home/pynaoqi/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages"

#WORKDIR /workspace
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd home/workspace;catkin_make'
RUN echo "source home/workspace/devel/setup.bash" >> ~/.bashrc
RUN chmod +x /home/workspace/src/nao_interaction/nao_audio/nodes/nao_audio.py /home/workspace/src/pepper_perception/src/plugins_pepper_ros.py /home/workspace/src/pepper_robot/pepper_sensors_py/nodes/laser.py
RUN chmod +x /home/initialization/bringup.sh  /home/initialization/nao_audio.sh  /home/initialization/quick_start.sh  /home/initialization/start_perceptions.sh  /home/initialization/stop.sh

# change launch config /opt/ros/kinetic/share/naoqi_driver/share
