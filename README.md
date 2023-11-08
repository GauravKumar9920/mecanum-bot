# Robot-alog

This repo contains about the code set for alog robot(A four-wheeled mecanum bot following non-holonomic constrains) - stand alone API for Navigation process. Detailed Documentation can be found in /Documents folder.

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
