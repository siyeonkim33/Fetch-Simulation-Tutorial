# Fetch Robot Manipulation test 
A guideline for Fetch robot under the simulation and real-world.

Tested on Ubuntu 18.04 + Ros Melodic

## Build and Compile
1. Clone this repository:
~~~
cd ~/Desktop
git clone https://github.com/siyeonkim33/fetch_ws.git
~~~
2. Build the workspace:
~~~
cd ~/Desktop/fetch_ws
catkin_make
~~~

## Execution

### Gazebo world
Before opening gazebo world, there might be several minor errors.
1. Error in REST request
change initial url in the configuration file
~~~
sudo gedit ~/.ignition/fuel/config.yaml 
change to "url: https://api.ignitionrobotics.org"
~~~

### PoseCNN
~~~
roscd pose_cnn
./experiments/scripts/ros_ycb_object_test.sh {gpu_ids}
~~~
