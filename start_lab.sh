xhost +local:docker

docker run -ti --rm \
       -p 8888:8888 \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v /home/tca/Workshop/Notebooks/ROS_JupyterLab/ipython_robot_prototyping:/root/catkin_ws/src/ipython_robot_prototyping \
       jupyter_lab
