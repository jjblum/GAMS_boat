#! /bin/bash

echo Syncing to $1
rsync /home/odroid/geographiclib-code/src/.libs/libGeographic.so.17.0.0 root@192.168.1.12$1:/usr/lib/

ssh root@192.168.1.12$1 "ln /usr/lib/libGeographic.so.17.0.0 /usr/lib/libGeographic.so.17"
ssh root@192.168.1.12$1 "ln /usr/lib/libGeographic.so.17.0.0 /usr/lib/libGeographic.so"
rsync /etc/udev/rules.d/10-boat.rules root@192.168.1.12$1:/etc/udev/rules.d/10-boat.rules

echo Synced to $1
