# Mocap

This package contain every scripts to get the position of the quadcopter from Qualisys or from Gazebo.

* Enter `sim` or `real` in Bash.
* Launch the mocap script: roslaunch mocap mocap.launch`.
* Add a body to track: see the [mocap.launch](launch/mocap.launch).
* Change the IP of the Qualisys computer: [scripts/mocap_source.py#L14](scripts/mocap_source.py#L14) (line 14).

# Troubleshoot
If the mocap connection fail, check if the Qualisys software is still responding. Otherwise, restart the software.

Check if the body identifier is the right one (see [mocap.launch](launch/mocap.launch)).
