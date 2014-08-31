#!/bin/bash

screen -d -m pd -nogui -audiodev 3 -noadc -outchannels 2 -audiobuf 15 /home/ccrma/curious/16chan/pd/curiousbio.pd
screen -d -m /home/ccrma/curious/16chan/main
