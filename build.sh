#!/bin/sh
docker build \
    --network host \
    --build-arg USER=developer \
    -t aw_universe \
    -f Dockerfile . "$@"