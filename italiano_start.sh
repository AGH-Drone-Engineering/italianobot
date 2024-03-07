#!/bin/bash

docker compose -f ~/compose.yaml up -d

python3 ~/ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/distance.py
