ROS-based controller for load transportation with iris+ quadcopters
===================================================================

A small guide to setting up and running this project.
This code is based on [this repository](https://github.com/antonioadaldo/quadcoptersSML).

## Install ROS

Follow the [official instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) for ROS Indigo.
After configuring the Ubuntu repositories and keys, this includes installing the full desktop install via
~~~~
sudo apt-get install ros-indigo-desktop-full
~~~~
(This might download more than 1GB of packages.)

Then initialize `rosdep`
~~~~
sudo rosdep init
rosdep update
~~~~

Make sure the ROS environment variables are set when bash-terminals are launched.
To do this, add `source /opt/ros/indigo/setup.bash` to the file `.bashrc`.
You can do this, by executing
~~~~
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~~
in a terminal window.

The installation of rosinstall (package `python-rosinstall`) is not required.
However the GUI depends on the package `pyqtgraph` which can be installed by running
```
sudo apt-get install python-pip
sudo pip install pyqtgraph
```
in a terminal.

## Setup Workspace

(For more detailed instructions see the [official ROS tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).)
Create a new folder and make sure it contains a folder named `src`.
Make sure this `src` directory is in the `ROS_PACKAGE_PATH` variable.
You can check this with `echo $ROS_PACKAGE_PATH`.
If the directory is missing (which is likely), add the line
~~~~
export ROS_PACKAGE_PATH=~/[your ros workspace]/src:${ROS_PACKAGE_PATH}
~~~~
to the file `~/.bashrc`.
For this to take effect, you should close and reopen the terminal or type `source ~/.bashrc`.

To clone the project files from GitHub, start a terminal window and navigate to the `src` folder.
Now run the following command (including the dot at the end!):
~~~~
git clone --recursive https://github.com/KTH-AEROWORKS/quad-suspended-load.git .
~~~~
to obtain the relevant files.
The `--recursive` option makes sure that the mavros packages are downloaded as a submodule.
If you have received this project as an archive, copy the relevant subfolders (currently `quad_control`, `gui` and possibly `mavros`) into the `src` folder.
Now you can initialize the ROS workspace by running `catkin_init_workspace` (within the `src` folder). 
If mavros is not included, create a folder and get the files from `https://github.com/mavlink/mavros`.

You should only have one mavros installation in your ROS path, i.e. you should not have mavros installed in another workspace or from the Ubuntu repositories (the mavros version that is currently available there is too old).

To make sure that mavros can access your USB port, you might need to add your user to the group `dialout` by executing `sudo usermod -a -G dialout $USER`.
(You might need to log out out and log in againt for this to take effect.)
Futhermore you need to install the control toolbox, which is required by mavros, by running the command `sudo apt-get install ros-indigo-control-toolbox` in a terminal.
You also have to install MAVLink, NumPy and SciPy if they are not already installed (use `sudo apt-get install ros-indigo-mavlink python-numpy python-scipy`).

## Build Project

Run `catkin_make` from the workspace root (the folder containing the `src` folder).

## Run Project

### Without Mavros (Simulation only)

In two terminal windows (both in the workspace root), run
~~~~
source ./devel/setup.bash
roslaunch quad_control iris1.launch
~~~~
and
~~~~
source ./devel/setup.bash
rqt --standalone  tabbedGUI --args Iris1/
~~~~

### With Mavros and One Quadcopter

If you want to connect to an actual quadcopter, run the following commands (the GUI is also started by the launch file):
~~~~
source ./devel/setup.bash
roslaunch quad_control single_quad_mavros.launch
~~~~

### With Mavros and Two Quadcopters

To start the controller for two quads, run:
~~~~
source ./devel/setup.bash
roslaunch quad_control dual_quad_mavros.launch
~~~~

### Start Quadcopter

Startup procedure for quadcopter controlled with ROS:

- Start ROS with mavros as described above.
- Plug in the battery of the quadcopter.
- Select and set a trajectory.
- Press "Min Throttle".
- Press the safety button on the quadcopter to activate the motors, then arm the quad.
- Select and set a controller (it uses the currently set trajectory).

## Uninstall ROS

Only if needed:

To uninstall ROS and remove all configuration files, execute the following commands
~~~~
sudo apt-get purge ros-indigo*
sudo apt-get purge python-rosdep python-rospkg python-rosinstall
sudo apt-get autoremove
~~~~
and remove all lines concerning ROS from `~/.bashrc`.
