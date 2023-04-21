# Docker for iiwa_ros

This folder holds a docker for iiwa_ros. This is an out of the box docker infrastructure that allows you to build an image with ros melodic or noetic containing iiwa_ros and all its dependencies, and sharing the network of the host computer.
## Prerequisite

### OS

**Linux:** This docker was developed to be used under Ubuntu. It has been tested on Ubuntu 20.04 but should work with any reasonably recent version. It should also be usable on other Linux-based os but will probably require some adaptations.

**OS-X:** For OSX users, the docker should work. However it is to be expected that the graphical interfaces such as Gazebo will not be working. This might be fixable for specific cases but support is not provided by this package. This is due to the fact that solutions to this may vary a lot depending on the hardware and versions. Also `install_docker.sh` will most likely not work on OS-X. However the user should be able to recreate by hand the necessary steps.

**Windows:** This will not work under windows. However I would suggest using [Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/about) which should allow the user to run this docker on a windows computer. However display sharing issues are still to be expected.

### Network

The docker container will share the network of the host. So if you plan to use it to communicate with the IIWA robot, you should first make sure that your host network and ports are correctly configured. Check the main README.md for more information about that.

### Nvidia GPU

Some graphic application (e.g: gazebo9) might need explicit access to your NVIDIA GPU to run without error if you have one. So if you have a NVIDIA GPU, make sure that you have installed a driver and that it is up to date. If you encounter issues with it, try upgrading to the latest LTS driver supported by you GPU.
Not that a NVIDIA GPU is not required to run this. The docker should work well even if you don't have one.

## Installation

To install the necessary dependencies to run this docker you can either do it manually or execute `install_docker.sh` which will check take you through the installation steps.

### Short cut (recommended)

Run:
```bash
bash install_docker.sh
```

This will check which commands are already on your system and propose you to install the missing ones. Unless you have a specific reason to decline, it is strongly suggested to say yes to all installations.

### Manual installation

If for some reasons you are not able to use the install script, you can install the dependencies by hand. 

1. This package requires you to have docker installed. More information about this can be found [here](https://docs.docker.com/engine/install/). If you are on Linux don't forget to run the [post install steps](https://docs.docker.com/engine/install/linux-postinstall/)

2. In addition the `start_docker.sh` script uses a command form [aica-technology/docker-images](https://github.com/aica-technology/docker-images) to cut down on code. To install it download the repo and execute `scripts/install-aica-docker.sh`. This is also a neat command to use is you are developing other dockers.
3. If you have a NVIDIA GPU you will also need to install `nvidia-docker2`. Installation instruction can be found [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Building image

To build the docker image simply run:
```bash
bash build_docker.sh
```

This script has a few options that can be used to specify various build variations. The options are:
```bash
Usage: ./build.sh [-r, --rebuild] [-v, --verbose] [-i, --image-name] [-d, --distro] [--smid]
Build the epfl-lasa/iiwa_ros  image.
Options:
  -r, --rebuild            Rebuild with --no-cache option.
  -v, --verbose            Display additional info upon build.
  -i, --image-name         Defines the name given to the image beeing built. Default: epfl-lasa/iiwa_ros
  -d, --distro             Can be \"noetic\" or \"melodic\", default: noetic
  --smid                   Build with smid flags on. Flags are off by default.
  -h, --help               Show this help message."
```

## Spinning container

To run the container, use:
```bash
bash start_docker.sh interactive
```

This will open the container command prompt. The user is `ros`, and starts in a ros workspace which is already built and sourced. So you might for example directly input a command such as:
```bash
roslaunch iiwa_gazebo iiwa_gazebo.launch
```

### Modes

`start_docker.sh` offers 3 modes. 

- `interactive`: Opens a prompt connected to the container. Stops the container as soon as the prompt is closed
- `server`: Starts the container in the background. The container will only be stopped when stopped explicitly or when `start_docker` is called again with interactive or server. (We always stop the previous docker before starting it again)
- `connect`: Allows to connect to the container which was started with the server command. This opens a prompt to the container. You can have as many prompt connected to a server as you want.

### Options

In addition a few options can be used when using  `start_docker`:

```bash
Usage: ./start_dockers.sh [interactive | server | connect] [-i, --image] [-u, --user]
Build the '${IMAGE_NAME}' image.
Options:
  interactive            Spin the image in the console
  server                 Spin the image as an ssh server
  connect                Connects to an active container
  -i, --image            The name of the image to use to start the container
  -u, --user             Specify the name of the login user. (optional)
  -h, --help             Show this help message and the one from aica-docker
  Additional arguments are passed to the aica-docker command.
```

## Technical notes

- The docker builds on top of docker images from the [aica-docker/docker-images](https://github.com/aica-technology/docker-images). The install script checks out the  And the `start_docker.sh` script uses their set of command. This might not be the best approach but this works for now so it is good enough. At the time of merging the latest commit working with these script is [5f8d908](https://github.com/aica-technology/docker-images/commit/5f8d9084b2d7b4fb69631b9e69aa72f64d7f7843) on main. Although it is likely to work with later commits. Note that the install script will install the `aica-docker` command from this commit.
- The tested base images versions are:
  - [ros-ws:melodic](https://github.com/aica-technology/docker-images/pkgs/container/ros-ws): sha256:d1ab8c7a5e28dd3fa71ebe051ac0937207adf03215235386a1a0edea855d68d2
  - [ros-ws:noetic](https://github.com/aica-technology/docker-images/pkgs/container/ros-ws): sha256:6c439fedd5b475995187a7206b8cd6f0f2651b809dd689aba5641f3975c61b7f
- When building the docker the iiwa-ros repository is cloned into the image. A better approach would probably be to copy it from the host computer. This would likely save a little time at build (who ever does that should also make sure that the .dockerignore is also updated)
- `start_docker.sh` checks for NVIDIA GPU and automatically chooses to enable it for the container if it is there. The reason is that it seems gazebo has some issues if there is only an NVIDIA GPU and the container does not have access. But it might still be nice to have an option to override this automatic detection. Also, the output of the command is printed on the terminal, we could do without. Maybe check if there is an NVIDIA driver instead of a physical GPU?
- In my case gazebo still outputs an error on the terminal but everything ends up working fine. I think this is partly because gazebo9 and gazebo11 are fairly old. So if everything else works just ignore error messages.
- Note that through the `build_docker.sh` you will forward your git user name and git email to the docker image. This will allow you to push and pull in the docker.
- When you build the docker using SIMD flags, the .bashrc in the image is updated to include `export ENABLE_SIMD=ON`. This enables SIMD for iiwa_ros without having to write it explicitly at every catkin_make. Just know it's there.
- `start_docker.bash` start the docker with `--net=host` which means the container has the same ip as the host. This enables to share ros communication seemlessly in, out and between containers.
- Containers names are hard coded in `start_docker.sh`. If you want multiple ones on the same machine you need to change manually the container name in the script... Well it's not practical, I didn't think of that... But in the same time, why would you want to do that? Feel free to fix it with an new option to specify container name it you run into this use case

## Authors/Maintainers

- Loïc Niederhauser: loic.niederhauser@epfl.ch, @niderha on github.
