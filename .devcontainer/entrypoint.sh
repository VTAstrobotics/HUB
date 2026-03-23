#!/bin/bash

# Set up logging entrypoint stuff to build.log file in .devcontainer directory
rm /workspace/.devcontainer/build.log
sudo touch /workspace/.devcontainer/build.log
sudo chmod 777 /workspace/.devcontainer/build.log

{ printf "entrypoint.sh:\n"
###############################################################################
#                                                                             #
# Startup commands (do not put anything above this)                           #
#                                                                             #
###############################################################################

# Set up git mergetool cli
git config merge.tool vimdiff
git config merge.conflictstyle diff3
git config mergetool.prompt false

# ROS basic build and source
cd /workspace/hub_ws
source build_scripts/build.sh

# Give permissions to input devices, like Xbox controller
printf "[INFO]: If you get a message like \"chmod: cannot access '/dev/input/js*': No such file or directory\" do not worry. If it ALSO says \"chmod: cannot access '/dev/input/event*': No such file or directory\" AND you have a controller plugged in, there may be an issue with seeing the controller. Note that the new Jetson Orin Nanos require a manual kernel patch to see Xbox controllers.\n"
sudo chmod +rx /dev/input/event*
sudo chmod +rx /dev/input/js*


###############################################################################
#                                                                             #
# Shutdown commands (do not put anything below this)                          #
#                                                                             #
###############################################################################
} &> /workspace/.devcontainer/build.log

exit 0