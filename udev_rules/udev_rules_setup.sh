#!/bin/bash
sudo cp 99-camera1394.rules /etc/udev/rules.d/
sudo addgroup video
sudo adduser $USER video