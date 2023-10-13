#!/bin/bash

# Name and base and options
IMAGE_NAME=epfl-lasa/iiwa_ros_mb_devel2                             # Chose any name for your image (but make sure to report it in start_docker)
ROS_DISTRO=noetic                                         # Possible: noetic, melodic
USE_SIMD=OFF                                              # Possible: ON, OFF
BASE_IMAGE=ghcr.io/aica-technology/ros-ws:${ROS_DISTRO}   # Do not modify

# Help
HELP_MESSAGE="Usage: ./build.sh [-r, --rebuild] [-v, --verbose] [-i, --image-name] [-d, --distro] [--smid]
Build the '${IMAGE_NAME}' image.
Options:
  -r, --rebuild            Rebuild with --no-cache option.
  -v, --verbose            Display additional info upon build.
  -i, --image-name         Defines the name given to the image beeing built. Default: ${IMAGE_NAME}
  -d, --distro             Can be \"noetic\" or \"melodic\", default: ${ROS_DISTRO}
  --smid                   Build with smid flags on. Flags are off by default.
  -h, --help               Show this help message."

# Parse build flags
BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
  -r | --rebuild)
    BUILD_FLAGS+=(--no-cache)
    shift 1
    ;;
  -v | --verbose)
    BUILD_FLAGS+=(--progress=plain)
    shift 1
    ;;
  -i | --image-name)
    IMAGE_NAME=$2
    shift 2
    ;;
  -d | --distro)
    ROS_DISTRO=$2
    if [[ "$ROS_DISTRO" != "noetic" && "$ROS_DISTRO" != "melodic" ]] ; then
      echo -e "\033[31mERROR: Distro \"$ROS_DISTRO\" is not supported"; \
      exit 1;
    fi
    shift 2
    ;;
  --smid)
    USE_SIMD=ON
    shift 1
    ;;
  -h | --help)
    echo "${HELP_MESSAGE}"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done


# Try to pull image
if [[ "$(docker images -q ${BASE_IMAGE} 2> /dev/null)" != "" ]]; \
  then echo "Base image already exists on the computer, proceeding to build docker";
  else echo -e "Base image does not exists ont the computer, pulling it..."; \
    docker pull "${BASE_IMAGE}" || echo -e "\033[31mCould not pull docker image ${BASE_IMAGE}";
fi

# Setup build flags
BUILD_FLAGS+=(--build-arg ROS_DISTRO="${ROS_DISTRO}")
BUILD_FLAGS+=(--build-arg USE_SIMD=${USE_SIMD})
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${ROS_DISTRO}")
BUILD_FLAGS+=(--build-arg HOST_GID=$(id -g))   # Pass the correct GID to avoid issues with mounted volumes
BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}")
BUILD_FLAGS+=(--build-arg GIT_NAME=$(git config user.name))    # Pass git user info to be able to pull
BUILD_FLAGS+=(--build-arg GIT_EMAIL=$(git config user.email))

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f ./docker/Dockerfile .