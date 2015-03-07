# Advanced Robotics

Shared code for CSE 668 Advanced Robotics

Code to get the Creative Senz3D camera working and display the 3D input in the form of a point cloud.


## Install

### Install PCL

	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all

### Install DepthSense SDK

* Create an account on http://www.softkinetic.com
* Go to Support > Download > DS311 & DS325 DepthSense SDK Linux > 64 bit 
* Download DepthSenseSDK-1.4.5-2151-amd64-deb
* Open a terminal
* Enter `cd /home/[user]/Downloads`
* Enter `chmod +x DepthSenseSDK-1.4.5-2151-amd64-deb.run`
* Enter `sudo ./DepthSenseSDK-1.4.5-2151-amd64-deb.run`

## Run

* Navigate to advanced-robotics/viewer/build
* Enter `cmake ..`
* Enter `make`
* Run `./viewer`