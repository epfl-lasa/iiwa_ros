#!/bin/bash
set -e

# setup environment
source $HOME/.bashrc

# start in home directory
cd $HOME/ros_ws

exec bash -i -c $@