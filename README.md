# Robot-alog

This repo contains about the code set for alog robot(A four-wheeled mecanum bot following non-holonomic constrains) - stand alone API for Navigation process. Detailed Documentation can be found in /Documents folder.

# How to use this repo

Clone the repo, Run odom_custom.launch for the bot to be publish it's odomery data, following which you can run any of the scripts or launch files.

move_base goes to the initial Starting postion, .traj_trac.launch gernates random waypoints and genrates a smooth curve following the waypoints, this genrated trajectory is followed by the bot.

Ps- Most of the scripts and it's usage documentaion can be found in the documentation folder along with the repository achitecture.

All the testing videos and SS can be found in the porject_report.pdf resources part(in the end)

Trajecotry-
![Bot-Trajectory](https://github.com/GauravKumar9920/mecanum-bot/blob/main/Documents/ss/navpath.png)

Holonomic-Bot-
![Holonomic-Bot](https://github.com/GauravKumar9920/mecanum-bot/blob/main/Documents/ss/Alogmini.jpeg)


