#!/bin/bash

docker buildx build --rm --platform linux/arm64 -t elijahanghw/ros_imav -f ./docker/ros_imav.dockerfile --load .
