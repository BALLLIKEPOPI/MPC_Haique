#!/bin/zsh

gnome-terminal -t "roscore" -x zsh -c "roscore;exec zsh;"
sleep 3s

gnome-terminal -t "acfly_mavros" -x zsh -c "source devel/setup.zsh;sudo chmod 777 /dev/ttyACM0;roslaunch mavros acfly.launch;exec zsh;"
sleep 1s

# for communication testing
# gnome-terminal -t "communicate" -x zsh -c "source devel/setup.zsh;rosrun communicationtest communicationtest_node ;exec zsh;"
# sleep 1s

gnome-terminal -t "MPC_run" -x zsh -c "source devel/setup.zsh;rosrun mpc_control mpc_control_node;exec zsh;"