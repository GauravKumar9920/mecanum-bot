# Robot-alog

This repo contains about the code set for alog robot(A four-wheeled mecanum bot following non-holonomic constrains) - stand alone API for Navigation process. Detailed Documentation can be found in /Documents folder.

# How to use this repo

Clone the repo, Run odom_custom.launch for the bot to be publish it's odomery data, following which you can run any of the scripts or launch files.

move_base goes to the initial Starting postion, .traj_trac.launch gernates random waypoints and genrates a smooth curve following the waypoints, this genrated trajectory is followed by the bot.

Ps- Most of the scripts and it's usage documentaion can be found in the documentation folder along with the repository achitecture.

All the testing videos and SS can be found in the porject_report.pdf resources part(in the end)
# Repo structure 

Mecanum-Bot/
├─ Alog-Control/
│  ├─ launch/
│  │  ├─ gmapping_slam.launch
│  │  ├─ joystick.launch
│  │  ├─ mapping.launch
│  │  ├─ move_base.launch
│  │  ├─ navigation.launch
│  │  ├─ odom_custom.launch
│  │  ├─ startup.launch
│  │  ├─ traj_trac.launch
│  ├─ rviz/
│  ├─ scripts/
│  │  ├─ base_move.py
│  │  ├─ gnav2.py
│  │  ├─ go_to_goal.py
│  │  ├─ vel_pub.py
│  │  ├─ waypoints_controller_traj_gen.py
│  │  ├─ waypoints_controller.py
│  │  ├─ waypoints_traj_controller.py
│  ├─ src/
├─ Documents/
├─ Embedded_communication/
├─ holonomic_controller/
├─ rplidar_ros/
├─ stop_recovery/
