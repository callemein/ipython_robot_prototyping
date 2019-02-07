FROM ros:kinetic-robot-xenial
SHELL ["/bin/bash", "-c"]

RUN apt update -y && apt full-upgrade -y 
RUN apt install -y ros-kinetic-desktop-full=1.3.2-0*

# We are following the instructions by:
# https://github.com/lidkalee/ipython_robot_prototyping


# Install offline  Robot Simulators
RUN apt install -y mpg123 python python-pip python3 python3-pip python-virtualenv

# Setup the folder containing the catkin workspace
RUN . /opt/ros/kinetic/setup.bash && mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make
RUN git clone https://github.com/callemein/ipython_robot_prototyping /root/catkin_ws/src/ipython_robot_prototyping
RUN . /opt/ros/kinetic/setup.bash && cd ~/catkin_ws/ && catkin_make

# This will create the virtualenv
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "--login", "-c"]

RUN pip3 install --upgrade pip setuptools
RUN pip3 install -r /root/catkin_ws/src/ipython_robot_prototyping/requirements.txt
RUN pip3 install jupyter_core \
                jupyter_client \
                jupyter_contrib_nbextensions \
                ipywidgets \
                jupyterlab \
                rospkg \
                chatterbot \
                pytz \
                gtts

RUN pip3 install -U `pip3 list --outdated | tail -n +3 | awk '{print $1}'`
RUN python3 -m jupyter nbextension enable --py --sys-prefix widgetsnbextension


ENTRYPOINT `source /root/catkin_ws/devel/setup.bash && roslaunch ipython_robot_prototyping simulators.launch `& \
	         `source /root/catkin_ws/devel/setup.bash && export SHELL=/bin/bash && \
           jupyter lab --allow-root --no-browser  --NotebookApp.token='' --no-browser --ip=0.0.0.0 --notebook-dir="~/catkin_ws/src/ipython_robot_prototyping/"`
