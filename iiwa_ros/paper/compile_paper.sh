#!/bin/bash

if [ ! -d "paper" ]
then
    echo -e "This script should be run in the iiwa_ros package folder ('roscd iiwa_ros'). Exiting."
    exit -1
fi

docker run --rm \
    --volume $PWD/paper:/data \
    --user $(id -u):$(id -g) \
    --env JOURNAL=joss \
    openjournals/inara \
    -o preprint,pdf,crossref \
    paper.md
