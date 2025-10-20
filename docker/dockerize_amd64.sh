#!/bin/bash

docker buildx build --rm --platform linux/amd64 -t elijahanghw/ros_imav -f ./docker/ros_imav.dockerfile --load .