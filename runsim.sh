#!/bin/sh

roslaunch flatland_server server.launch world_path:="$(pwd)/c_world/world.yaml"
