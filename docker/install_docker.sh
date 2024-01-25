#!/bin/bash

# If not installed, suggest install docker
if [ ! command -v docker &> /dev/null ]; then

    while true; do
        read -p "Docker command not available, do you whish to install it (y/n)?" yn
        case $yn in
            [Yy]* ) 

                # Docker installation according to https://docs.docker.com/engine/install/ubuntu/
                echo "Docker not found, installing..."
                sudo apt-get update
                sudo apt-get install ca-certificates curl gnupg lsb-release
                sudo mkdir -p /etc/apt/keyrings
                curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
                echo \
                    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
                    $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
                sudo apt-get update
                sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
                echo "Docker installed"
                break;;

            [Nn]* ) 
                echo "Skipping docker installation. Note that it will likely impair your ability to run dockers"
                break;;

            * ) echo "Please answer y or n";;
        esac
    done
else
    echo "Docker already found, skipping install."
fi


# Check docker post install
if ! [ $(getent group docker) ] || ! id -nG | grep -qw "docker" ; then # ! [ id -nG | grep -qw "docker" ]; then

    while true; do
        read -p "Docker post install steps for Linux not done, do you whish to do it (y/n)?" yn
        case $yn in
            [Yy]* ) 
                # Docker post-installation according to https://docs.docker.com/engine/install/linux-postinstall/
                echo "Running docker post install..."
                
                # Create docker group if not already done
                if ! [ $(getent group docker) ] ; then
                    sudo groupadd docker
                fi

                # Add user to docker group
                sudo usermod -aG docker $USER
                newgrp docker
                break;;
            [Nn]* ) 
                echo "Skipping docker post install steps. Without it this user might have to call docker with sudo everytime."
                break;;
            * ) echo "Please answer y or n";;
        esac
    done

else
    echo "Docker post install already done. Skipping step."
fi

# If nvidia, install nividia-docker2
if [[ $(sudo lshw -C display | grep vendor) =~ NVIDIA ]]; then
    if [ $(dpkg-query -W -f='${Status}' nvidia-docker2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then

        while true; do
            read -p "You have a nvidia graphic card. Do you wish to install nvidia-docker2 to use it in your container (y/n)?" yn
            case $yn in
                [Yy]* ) 
                    # nvidia-docker2 install according to: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
                    echo "Installing nvidia-docker2"
                    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
                        && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
                        && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
                        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
                    sudo apt-get update
                    sudo apt-get install -y nvidia-docker2
                    sudo systemctl restart docker
                    echo "Installation done."
                    break;;
                [Nn]* ) 
                    echo "Skipping install of nividia-docker2. Note that this might create some issues with the GUI of your application in the container."
                    break;;
                * ) echo "Please answer y or n";;
            esac
        done
    else
        echo "nividia-docker2 already installed, skipping this step."
    fi
else
    echo "No nvidia graphic cards detected, skipping install of nvidia-docker2."
fi


# Install aica docker
if [ ! command -v aica-docker &> /dev/null ]; then
    while true; do
        read -p "aica-docker command not found, do you whish to install it? It will install the latest version that worked with this script at the time of developpement. (y/n)?" yn
        case $yn in
            [Yy]* ) 
                echo "Installing aica-docker..."
                git clone git@github.com:aica-technology/docker-images.git /tmp
                git --git-dir=/tmp/docker-images/.git checkout 5f8d908
                sudo bash /tmp/docker-images/scripts/install-aica-docker.sh
                rm -rf /tmp/docker-images
                echo "aica-docker installed."
                break;;
            [Nn]* ) 
                echo "Skipping installation of aica-docker. This might impair the start docker scripts execution."
                break;;
            * ) echo "Please answer y or n";;
        esac
    done
else
    echo "aica-docker already found, skipping install."
fi


