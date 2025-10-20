#!/bin/bash

docker buildx build --platform linux/amd64 -t elijahanghw/ros_imav -f ./docker/ros_imav.dockerfile --load .