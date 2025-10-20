#!/bin/bash

# docker buildx build --platform linux/arm64,linux/amd64 -t elijah/ros_imav -f ./docker/ros_imav.dockerfile --load .

docker buildx build --platform linux/amd64 -t elijah/ros_imav -f ./docker/ros_imav.dockerfile --load .