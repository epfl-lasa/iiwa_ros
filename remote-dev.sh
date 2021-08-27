#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=iiwa_ros
STAGE_NAME=dev-user
REBUILD=0

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

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/../" \
    --opt o="bind" \
    "${IMAGE_NAME}_ros_ws_vol"

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="${IMAGE_NAME}_ros_ws_vol:/home/ros/ros_ws/src/iiwa_ros" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${XAUTHORITY}:${XAUTHORITY}" \
  --env XAUTHORITY="${XAUTHORITY}" \
  --env DISPLAY="${DISPLAY}" \
  "${IMAGE_NAME}:${STAGE_NAME}"