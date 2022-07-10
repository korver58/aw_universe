#!/bin/bash
docker run --rm -it \
    --privileged \
    --net=host \
    --volume=$(pwd):/home/developer/host_dir \
    --env="DISPLAY=${DISPLAY}" \
    --env="HOSTIP=${HOSTIP}" \
    aw_universe:latest