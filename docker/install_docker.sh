#!/bin/bash

# If not installed, install docker with post install

# If nvidia, install nividia-docker2

# Install aica docker
if [ ! command -v aica-docker &> /dev/null ]; then
    echo "Installing aica-docker..."
    git clone git@github.com:aica-technology/docker-images.git /tmp
    sudo bash /tmp/docker-images/scripts/install-aica-docker.sh
    rm -rf /tmp/docker-images
    echo "aica-docker properly installed"
else
    echo "AICA docker already found"
fi


