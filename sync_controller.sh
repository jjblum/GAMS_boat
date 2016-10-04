#! /bin/bash

echo Syncing to boat $1

rsync /home/odroid/Documents/boat_agent/custom_controller odroid@192.168.1.12$1:Documents/boat_agent/
ssh odroid@192.168.1.12$1 "kill -9 \`cat /home/odroid/tmp/boat_controller.pid\`"

echo Syncing complete for boat $1
