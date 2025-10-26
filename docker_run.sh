#!/bin/bash

# sudo docker run -it --rm --network=host elijahanghw/ros_imav:latest

sudo docker run -it --rm \
  --network=host \
  --privileged \
  --device /dev/gpiomem4 \
  --device /dev/mem \
  elijahanghw/ros_imav:latest
