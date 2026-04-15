#!/bin/bash

# This script copies your entire ANUBIS directory to a remote robot
# over SSH using rsync. It assumes you have SSH access to the robot.
# This means that you do not need to connect the robot to the internet or to a monitor 
# unless you are adding external dependencies. The syncing should handle any amount of changes you may be making
# note that the ANUBIS directory on the robot will be replaced with your local copy.
# and the Anubis Directory on the robot will be located at ~/Deployments/ANUBIS
# The normal Anubis directory will be left alone for any potential edge cases.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ANUBIS_DIR="$( dirname "$SCRIPT_DIR" )"   # parent of deploy

Dest_IP="$1"

if [ -z "$Dest_IP" ]; then
  echo "Usage: $0 user@host:/path"
  echo
  echo "Description:"
  echo "  user@host:/path  The SSH destination where the ANUBIS directory"
  echo "                   will be synced. Example:"
  echo "                   ./sync.sh rover@192.168.1.50"
  exit 1
fi



Dest="$Dest_IP":/home/astrobotics/Deployments/HUB

ssh "$Dest_IP" "mkdir -p /home/astrobotics/Deployments/HUB"

rsync -avz --delete --exclude "build/" --exclude "log/" --exclude "install/" "$ANUBIS_DIR/" "$Dest/"
