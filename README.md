# wall_following_algothm_drone

copy in catkin_ws

open terminal

navigate to catkin_ws

run "catkin_make" command

run "source devel/setup.bash"


1) 1st Terminal

roslaunch drone_description drone.launch

2) 2nd Terminal

rosservice call /enable_motors "enable: true"

rosrun drone_description navigate.py

3) 3rd Terminal
rosrun drone_description graph.py


https://www.youtube.com/watch?v=9AVZa_sX_r4&t=1s
