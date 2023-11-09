# Robot-alog

This repo contains about the code set for alog robot(A four-wheeled mechanum bot following non-holonomic constraints) - stand alone API for the Navigation process. Detailed Documentation can be found in /Documents folder.

# How to use this repo

Clone the repo, and Run odom_custom.launch for the bot to publish its odomery data, following which you can run any of the scripts or launch files.

move_base goes to the initial Starting position, .traj_trac.launch generates random waypoints and generates a smooth curve following the waypoints, this generated trajectory is followed by the bot.

Ps- Most of the scripts and their usage documentation can be found in the documentation folder along with the repository architecture.

All the testing videos and SS can be found in the porject_report.pdf resources part(in the end)

Trajectory-

![navpath](https://github.com/GauravKumar9920/mecanum-bot/assets/84905312/5bf5ae92-150e-414c-a8be-c5f30bc0ffb9)


Holonomic-Bot-
![Alogmini](https://github.com/GauravKumar9920/mecanum-bot/assets/84905312/31275b1a-3af4-457d-bb18-716a150ffaea)

Optically Tracking trajectory using fiducial Markers-
![IMG_20230711_125456463-min](https://github.com/GauravKumar9920/mecanum-bot/assets/84905312/9de589ca-8756-44d5-b1e3-16d5e7774d8f)

Bot following a Provided Trajectory - 

https://github.com/GauravKumar9920/mecanum-bot/assets/84905312/e31a00a7-59d1-471a-a564-6e232efb81c5


