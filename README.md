# 📍 amr_ws

This repository contains the complete **ROS 2 Humble** workspace of our **Final Year Engineering Project**<br> 
— an **Autonomous Mobile Robot** developed for educational and research purposes.

---

## 🛠️ Design & Development of Autonomous Mobile Robot (AMR)

## 🎯 Project Highlights:-

- All AMR processes are controlled/operated through laptop via **SSH**
- **Autonomous Navigation** with real time **Obstacle Avoidance** is done by using open source ROS 2 **Nav2** stack
- **SLAM Mapping** with YDLiDAR X2 and RViz2 using open source ROS 2 **SLAM TOOLBOX** Package
- **Teleoperation** is done via keyboard
- **Encoder-based Odometry** is used for accurate localization 
- Custom Robot **URDF** is made for RViz Visualization
- All the commands are executed on "RPI terminals" via SSH except "Rviz & URDF" which is executed on "laptop terminal" to avoid the lagging and reduce the load on RPI. For this just keep "export ROS_DOMAIN_ID=0" same in .bashrc file of both Laptop & RPI

---

## ⚙️ Main Teck Stack Components:-     
-  Microprocessor - Raspberry Pi 4B with 4GB RAM 32GB SD Card
-  OS - **Ubuntu Desktop 22.04.5** is installed on Raspberry Pi
-  Entire project is based on **ROS 2 Humble**   
-  Microcontroller - Arduino Mega
-  Lidar sensor - YDLiDAR X2 
-  Qudrature Optical Rotary Encoders - Hall Effect based incremental encoders are used for odometry    
-  Motors - Johnson 12V 200 RPM DC geared motor-Grade A Quality-Encoder compatible   
-  Motor Driver - Cytron MDD10A Dual Channel Motor Driver Shield for arduino is used
-  Battery - 12.8V 12000 mAh Li-ion battery pack is used

---

## 🚀 How to build ws on your system:-
-  **If you have similar software & hardware setup on your AMR you can directly use this repo to build your workspace and have fun with your AMR**
-  Step 1- Copy the repo (src & config folder only) in your amr_ws then build all the packages available in "src" folder one by one to avoid any errors.
-  Step 2- Modify and configure the .yaml & launch files with correct file names & their paths as per you setup or requirement.
-  Step 3- Ensure you upload the correct code in your arduino which is available in this repository.  
-  Step 4- Do all the hardware & electronics connections properly and then test everything.
-  Step 5- Tune the parameters accordingly if your setup is a bit different.

--------------------------------------------------------------------

## 🧱 System Architecture in short

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
