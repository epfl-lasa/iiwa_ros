#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=iiwa_ros
STAGE_NAME=ros-user
REBUILD=0

path=$(echo "${PWD}" | rev | cut -d'/' -f-2 | rev)
if [ "${path}" != "iiwa_ros/docker" ]; then
  echo "Run this script from within the directory iiwa_ros/docker !"
  echo "You are currently in ${path}"
  exit 1
fi

while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done

BUILD_FLAGS=(--target "${STAGE_NAME}")

if [[ "${OSTYPE}" != "darwin"* ]]; then
  UID="$(id -u "${USER}")"
  GID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${UID}")
  BUILD_FLAGS+=(--build-arg GID="${GID}")
fi
BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${STAGE_NAME}")

if [ "${REBUILD}" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .. || exit

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${XAUTHORITY}:${XAUTHORITY}" \
  --env XAUTHORITY="${XAUTHORITY}" \
  --env DISPLAY="${DISPLAY}" \
  "${IMAGE_NAME}:${STAGE_NAME}"