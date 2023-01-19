#!/bin/bash

# Install aica docker
if ! command -v aica-docker &> /dev/null
then
    echo "Installing AICA docker"
    git clone git@github.com:aica-technology/docker-images.git /tmp
    sudo bash /tmp/docker-images/scripts/install-aica-docker-sh
    rm -rf /tmp/docker-images
else
    echo "AICA docker already found"
fi


