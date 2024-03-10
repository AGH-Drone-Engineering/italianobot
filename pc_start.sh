#!/bin/bash

# sshpass or -i ~/.ssh/id_rsa
export IP=192.168.1.106

sshpass -p 'husarion' ssh -i ~/.ssh/id_rsa husarion@$IP '. ~/ros2_ws/src/italianobot/italiano_start.sh'

ros2 launch itabot real.launch
