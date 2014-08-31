#!/bin/bash

# This doesn't work, don't know why
# Tried:
#   $ sudo ./run.sh
#   <enter password>

screen -d -m pd -nogui -noadc -outchannels 2 -audiobuf 15 8chan_drone.pd
screen -d -m ./main
