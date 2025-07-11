# 📍 amr_ws

This repository contains the complete **ROS 2 Humble** workspace of my **Final Year Engineering Project**<br> 
— an **Autonomous Mobile Robot** developed for educational and research purposes.

---

## 🛠️ Design & Development of Autonomous Mobile Robot (AMR)

## 🎯 Project Highlights :-

- All AMR processes are controlled/operated through laptop via **SSH**
- **Autonomous Navigation** with real time **Obstacle Avoidance** is done by using open source ROS 2 **Nav2** stack
- **SLAM** with YDLiDAR X2 and RViz2 is done by using open source ROS 2 **SLAM TOOLBOX** Package
- **Teleoperation** for testing, manual control & mapping is done via Keyboard. It can also be done by Joystic if you have one.
- **Encoder-based Odometry** is used for accurate localization 
- Custom Robot **URDF** is made for RViz Visualization
- All the commands are executed on "RPI terminals" via SSH except "Rviz & URDF" which is executed on "laptop terminal" to avoid the lagging and reduce the load on RPI. For this just keep **"export ROS_DOMAIN_ID=0"** same in .bashrc file of both Laptop & RPI.

---

## ⚙️ Main Teck Stack Components :-     
-  Microprocessor - Raspberry Pi 4B with 4GB RAM & 32GB SD Card
-  OS - **Ubuntu Desktop 22.04.5** is installed on Raspberry Pi
-  Entire project is based on **ROS 2 Humble**   
-  Microcontroller - Arduino Mega
-  Lidar sensor - YDLiDAR X2 (360 degree 2D Lidar with range 8m)
-  Qudrature Optical Rotary Encoders - Hall Effect based incremental encoders are used for odometry    
-  Motors - Johnson 12V 200 RPM DC geared motor-Grade A Quality-Encoder compatible   
-  Motor Driver - Cytron MDD10A Dual Channel Motor Driver Shield for arduino is used
-  Battery - 12.8V 12000 mAh Li-ion battery pack is used

---

## 🚀 How to build amr_ws on your system :-
-  **If you have similar software & hardware setup on your AMR you can directly use this repo to build your workspace and have fun with your AMR**
-  Step 1- Copy the repo (src & config folder only) in your amr_ws then build all the packages available in "src" folder one by one to avoid any errors.
-  Step 2- Modify and configure the .yaml & launch files with correct file names & their paths as per you setup or requirement.
-  Step 3- Ensure you upload the correct code in your arduino which is available in this repository.  
-  Step 4- Do all the hardware & electronics connections properly and then test everything.
-  Step 5- Tune the parameters accordingly if your setup is a bit different.

---

## 📁 Workspace Structure :-
amr_ws/<br>
├── src/<br>
│   ├── amr_node/<br>
│   ├── amr_description/<br>
│   ├── amr_navigation/<br>
│   ├── YDLidar-SDK/<br>
│   └── ydlidar_ros2_driver/<br>
└── config/<br>


---

## 📝 How to Run :-
- After setting up everything and building the workspace source it and use the below commands to run everything correctly.
- Step 1- "ros2 run amr_node amr_node_latest"	(run this command in 1st ssh terminal)"
- Step 2- "ros2 launch ydlidar_ros2_driver ydlidar_launch.py"		(run in 2nd ssh terminal to start lidar laser scaning)
- Step 3- "ros2 run teleop_twist_keyboard teleop_twist_keyboard"	(run in 3rd ssh terminal for teleoperation for mapping)
- Step 4- "ros2 launch amr_description display.launch.py"		(run in 1st laptop terminal to launch robot urdf model in rviz)
- Step 5- "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/adarsh/amr_ws/config/slam_params.yaml"	(run in 4th ssh terminal to start creating a 2D map using LiDAR)
- Step 6- Then teleoperate your AMR slowly in your environment to creat a better quality map.
- Step 7- "ssh rpi_hostname@rpi_ip "source /opt/ros/humble/setup.bash && source /home/adarsh/amr_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /home/adarsh/amr_ws/src/amr_navigation/maps/new_map_name_here"	(run in 2nd laptop terminal to save generated map on rpi)
- Step 8- After saving the map kill every terminal of both ssh and laptop 
- Step 9- Again launch amr node, lidar and navigation command on ssh terminal, and launch Rviz command on laptop terminal
- Step 10- "ros2 launch amr_navigation navigation_launch.py"	(run on ssh terminal, command to load saved map in Rviz and Launch navigation process)
- Step 11- Now set initial position of your AMR using "2D Pose Estimate" button in Rviz. Try to match real time laser scans with generated map.
- Step 12- Then give a goal to AMR using "2D Goal Pose" button in Rviz.
- Step 13- Now watch your AMR navigating autonomously and avoiding obstacles in real time. ENJOY!

## ✅ Ensure the Rviz Setup :-
- Refer the screenshots shared.

--------------------------------------------------------------------

## 🧱 System Architecture in short :-

```plaintext
+---------------------+                    +----------------------+
| Raspberry Pi (ROS 2)|       Serial       |   Arduino Mega       |
|                     | <----------------> |  (Motor Control &    |
|  - AMR_Node/teleop  |                    |  Encoder Feedback)   |
|  - SLAM / Nav2      |                    +----------------------+
|  - LiDAR Drivers    |					|       
+---------------------+					|
           |____________________________________________|
				 |
		 Controlled through Laptop via SSH		 
