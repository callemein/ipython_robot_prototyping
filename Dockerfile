FROM ros:kinetic-robot-xenial
RUN apt update -y && apt full-upgrade -y 

SHELL ["/bin/bash", "-c"]

RUN apt install -y ros-kinetic-desktop-full=1.3.2-0*

# We are following the instructions by:
# https://github.com/lidkalee/ipython_robot_prototyping


# Installing Python ROS Depends
#RUN apt install -y python3-rospy python3-rospkg python3-rosdep

# Install offline  Robot Simulators
RUN apt install -y mpg123
RUN apt install -y python3 python3-pip python-virtualenv

# Setup the folder containing the catkin workspace
RUN . /opt/ros/kinetic/setup.bash && mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make
RUN cd ~/catkin_ws/src/ && git clone https://github.com/callemein/ipython_robot_prototyping
RUN . /opt/ros/kinetic/setup.bash && cd ~/catkin_ws/ && catkin_make

RUN mkdir ~/virtenv
RUN virtualenv -p python3 ~/virtenv/ros

RUN . ~/virtenv/ros/bin/activate && cd ~/catkin_ws/src/ipython_robot_prototyping && pip3 install -r requirements.txt
RUN cd ~/catkin_ws/src/ipython_robot_prototyping && pip3 install -r requirements.txt
RUN . ~/virtenv/ros/bin/activate && pip install -U `pip list --outdated | tail -n +3 | awk '{print $1}'`
RUN . ~/virtenv/ros/bin/activate && pip install --upgrade pip
RUN . ~/virtenv/ros/bin/activate && pip install --upgrade jupyter_core jupyter_client
RUN . ~/virtenv/ros/bin/activate && pip install jupyter_contrib_nbextensions ipywidgets jupyterlab

RUN . ~/virtenv/ros/bin/activate && jupyter nbextension enable --py --sys-prefix widgetsnbextension
RUN . ~/virtenv/ros/bin/activate && pip install rospkg chatterbot

RUN cd ~/catkin_ws/src/ipython_robot_prototyping && pip3 install -r requirements.txt
RUN pip3 install chatterbot pytz gtts rospkg jupyter_contrib_nbextensions ipywidgets jupyterlab
RUN pip3 install jupyter jupyter_core
RUN pip3 install -U jupyter jupyter_core

RUN jupyter nbextension enable --py --sys-prefix widgetsnbextension

ENTRYPOINT `. ~/catkin_ws/devel/setup.bash && \
	   . ~/virtenv/ros/bin/activate && \
	   roslaunch ipython_robot_prototyping simulators.launch `& \
	   `. ~/catkin_ws/devel/setup.bash && \
	   . ~/virtenv/ros/bin/activate && \
	   cd ~/catkin_ws/src/ipython_robot_prototyping && jupyter lab --allow-root --no-browser  --NotebookApp.token='' --no-browser --ip=0.0.0.0 --notebook-dir="~/catkin_ws/src/ipython_robot_prototyping/"`
