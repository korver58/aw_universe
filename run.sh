#!/bin/bash
docker run \
    --rm -it \
    --gpus all \
    --privileged \
    --net=host \
    --volume=$(pwd):/home/developer/mount \
    --env="DISPLAY=${DISPLAY}" \
    --env="HOSTIP=${HOSTIP}" \
    aw_universe:latest
    # ghcr.io/autowarefoundation/autoware-universe:latest-prebuilt
