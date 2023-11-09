# Robot-alog

This repository contains the code set for Alog robot, a four-wheeled mecanum bot following non-holonomic constraints. The stand-alone API for navigation processes is documented in detail in the /Documents folder.

## How to use this repo

1. Clone the repo.
2. Run `odom_custom.launch` for the bot to publish its odometry data.
3. After that, you can run any of the scripts or launch files.

The `move_base` script goes to the initial starting position. The `.traj_trac.launch` script generates random waypoints and generates a smooth curve following these waypoints, which the bot then follows.

Note: Most of the scripts and their usage documentation can be found in the documentation folder along with the repository architecture.

All testing videos and screenshots can be found in the `project_report.pdf` resources part (at the end).

![Holonomin-Bot](https://github.com/GauravKumar9920/mecanum-bot/blob/main/Documents/ss/Alogmini.jpeg)

## Repo structure 

