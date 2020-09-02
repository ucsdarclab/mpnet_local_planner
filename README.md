# MPNet-Documentation

This repository aims to document the setup and usage of the Dynamic MPNet Planner for the purpose of robot navigation.

## System Requirements

The planner was tested on a Docker Container with the following specifications.
1. Docker version 19.03.8
3. Ubuntu 18.04
2. ROS-Melodic
3. LibTorch Build Version 1.5.1
4. CUDA Version 10.2
5. NVidia Driver Version: 440.59

## Docker Installation

If you do not have Docker installed, please follow the official instructions on their website.
   Link : https://docs.docker.com/engine/install/ubuntu/

## Installing Requirements

The following steps will get you started with the MPNet Planner.

1. Clone this repository to your system. Open up a terminal and type in 
   
   ```
   git clone https://github.com/venkateshprasad23/MPNet-Documentation.git
   ```

2. Build the image required by running the DOckerfile provided in the repository. \
   Open a terminal and type in the following command in the directory containing the Dockerfile.
   Note : this may take quite some time.
   
   ```
   sudo docker build - < Dockerfile -t mpnet_plan
   ```
3. Now spawn off a container using this image. Type in the following command  
   
   ```
   sudo docker run -it \
	--gpus all \
	-e DISPLAY=$DISPLAY \
	-v $XAUTH:/root/.Xauthority \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	--volume $HOME/MPNet-Documentation/data/:/root/data/ \
	--volume $HOME/MPNet-Documentation/project/:/root/project/ \
	--volume $HOME/MPNet-Documentation/ipopt_installation/:/root/ \
	mpnet_plan:latest \
	bash
   ```
   
 4. In order to install IpOPT, follow the steps below.
   
   ```
   cp install_ipopt.sh /opt/ipopt
   ```   
   
   ```
   cd /opt/ipopt
   ```    
   
   ```
   wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
   ```
   
   ```
   bash install_ipopt.sh ./Ipopt-3.12.7
   ```
   
 ## Package Installation
    
   ```
   cd
   source /opt/ros/melodic/setup.bash 
   cd project/
   catkin_make
   source devel/setup.bash
   ```
   
 ## Running the simulation
   
   ```
   roslaunch mpnet_plan simulate_mpnet.launch
   ```
   
   This should spawn off an instance of RViz, where you can set the 2D Nav Goal and see the mpnet planner at work!
 
   
   
   

