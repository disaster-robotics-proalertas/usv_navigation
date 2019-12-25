# usv_navigation

This repository should be install in a different catkin_workspace and compiled with the *catkin_make_isolated* command.

This branch currently provides manual control with gui (gui:=true) or joystick (usv_teleop) and automatic heading control.

usv_teleop provides direct control of sail and rudder joint angles.

## Teleop example

On branch usv_navigation of usv_sim_lsa:

```
        roslaunch usv_sim sailboat_scenario0.launch parse:=true
        roslaunch usv_sim sailboat_scenario0.launch parse:=false
        roslaunch usv_teleop sony_dualshock3.launch
```
