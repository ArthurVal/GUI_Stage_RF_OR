/==============================================================================/
/                       README file GUI_RIDDLE Ros package
/
/  Created by  : Arthur Valiente (CNRS-LAAS intern 2015) 
/  Contact     : valiente.arthur@gmail.com 
/
/  History : 
/       Person       |       Date       |        Modification   
/----------------------------------------------------------------------
/  Arthur Valiente   |    15/06/2015    |     Creation of the file  
/   
/ 
/
/==============================================================================/



		DESCRIPTION

This package has been create for the RIDDLE project at CNRS-LAAS of Toulouse.
Its goal is to have an easy way to control and observe both ORK vision algorithm and RF detection system through a simple GUI.
It communicates with RF_RIDDLE node to give him simple commands, read the outputs from RF_RIDDLE & ORK nodes and plots the result on an RVIZ windows.



		INSTALLATION GUIDE (/!\ catkin & git require /!\)

This node has been created & build on both Ubuntu 12.04 & 14.04 with ROS-hydro & ROS-indigo.
Other versions of Ubuntu & ROS haven't been tested yet. 

    Dependances & ROS packages needed for installation :
      ROS : 
       -> rf_riddle node (need to be installed before to create msg & srv for gui compilation)
       -> object_recognition_core
       -> RVIZ
       -> Roscpp
       -> Rospy
      other stuff :
       -> Qt Creator (v4.8 or higher)
       -> Qwt

    How to install:

  1- First, setup your workspace for installation.
Create or use an already existing catkin workspace wherever you want.

To create a new workspace (recommended) follow the instructions:
 ->Go where you want to put your catkin workspace and type the following commands on your terminal:
mkdir -p gui_riddle_ws/src
cd gui_riddle_ws/src
catkin_init_workspace

 -> You can build your workspace even if it is empty by doing the following:
cd gui_riddle_ws
catkin_make

 -> Within the <your_ws_name>/src/ folder, create a new directory, corresponding to the gui package where you will put all sources files:
cd gui_riddle_ws/src
mkdir gui_riddle && cd gui_riddle

 -> Don't forget to source the setup.bash file from catkin
source /path-to-your-folder/gui_riddle_ws/devel/setup.bash

 2- Download the sources files. Inside your "package" folder, git clone the sources files:
git clone https://github.com/ArthurVal/GUI_Stage_RF_OR.git

 3- Compile the ROS node:
cd ../.. (go to gui_riddle_ws/ )
catkin_make


		UTILISATION

rosrun gui_riddle gui_riddle

No options available.



		TECHNICAL INFORMATION


















