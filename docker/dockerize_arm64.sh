#!/bin/bash

docker buildx build --no-cache --platform linux/arm64 -t elijahanghw/ros_imav -f ./docker/ros_imav.dockerfile --load .