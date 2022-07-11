#!/bin/bash
docker run \
    --rm -it \
    --gpus all \
    --privileged \
    --net=host \
    --volume=$(pwd):/home/developer/aw_universe \
    --env="DISPLAY=${DISPLAY}" \
    --env="HOSTIP=${HOSTIP}" \
    aw_universe:latest
