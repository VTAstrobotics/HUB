# This is the code for Astrobotics 2026 Lunabotics robot, A.Z.R.A.E.L

## Deploying on the robot
Attach a battery and turn on the robot  

Then attach the other antenna to the computer you will use to control the robot 

Ensure that the wired connection is showing up properly  

Find the IP Address of robot using "insert website and steps here"
(The two antennas currently use `192.168.0.96`, while the big wifi router uses `192.168.0.102`. This could change)  

Run `ssh astrobotics@<ip_address>` in the terminal, replacing `<ip_address>` with the actual ip address of the robot  

You should now be connected to the robot's terminal remotely  

Navigate to the HUB workspace using `cd deployments/HUB`  

Swap to the desired branch and pull code if needed  

You can test updated code by running the build script using `source ./build_scripts/build.sh`

Start tele-op control by running `ros2 launch robot_launch teleop.launch.py`
