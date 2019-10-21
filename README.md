# sub-utilities
This repo serves as the top level repo for KSU's AUV autonomy. It utilizes something called [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). These submodules are a way to organize and modularize our code to be easily editable and deployable to multiple systems. Basically, it's another repo inside a repo.

Because of this submodule system, I would recomend cloning with the following:
```
git clone --recurse-submodules https://github.com/ksu-auv-team/sub-utilities.git
```
This will clone the repo and initialize the submodules automatically. 

If you have **not** cloned this way, you may also:
```
git submodule init
git submodule update
```

I can't stress enough how much a brief look through [this submodules tutorial](https://git-scm.com/book/en/v2/Git-Tools-Submodules) will help you if you are confued.

### Scripts
This repo also has nice startup scrips to aid in the useability of the submarine all located in the `scripts` directory. 

* `camera_node.py` will publish camera footage on the `imgs` topic from /dev/video0
* `vis.py` allows the user to listen on the `imgs` topic and show realtime video feed.
* `runsub.py` will run the startup process for all of AUV's different nodes including: roscore, movement\_package, execute\_witState, etc. This is the script you want to run if you are trying to run the full machine. 

### Submodules
Inside the `submodules` directory is where all the submodules of this repo live. Most of these are also required for making the sub work properly and you should checkout their README's to get a better understanding of what they all do.   

If you push a commit to a submodule, it will not ***Automatically*** show up here. You need to go into the `submodules/<your-updated-submodule>` directory, do a `git fetch`, then checkout the commit that you want. When you then return to the base `sub-utilities` directory, and do a `git status`, you should see it showing that you updated a submodule. You can then `git add` and `git commit` that like normal and it will update the `sub-utilities` repo. 

In this repo, we have a few submodules currently: 
 * **ncs-ros** - Legacy Neural Network stuff 
 * **network_wrapper** - The current way we run a neural network 
 * **subdriver** - The state machine and autonomy code
 * **movement_package** - The wrapper for driving the sub with PWM commands

### network\_wrapper
The network wrapper repo is what is in charge of taking the images we publish out from the scripts above and running neural network inference on them. It then published back out both the new images with boxes drawn, and the actual detection information. 

To use it for inference or training, you will need tensorflow installed on your system. There is a non-GPU-enabled version that's just called tensorflow. But we *Typically* want the GPU enabled one because it runs significantly faster. [Here's a link](https://www.tensorflow.org/install/gpu) to how to install tensorflow-gpu. Just be aware that it's not as simple as doing a `pip install`, you will need to install some CUDA enabled stuff and probably update your video drivers. It's a hastle, but a requirement to use the GPU enabled stuff. One word of advice, **really make sure you've got the right versions**. 

### ROS dependencies for subdriver & movement\_package
Subdriver is what we use to control the high level architechture of the sub. It's what we use to control the state machine, write joystick control, publish out health data, etc. This is where you should go to work on new autonomy code changes.

The main dependency we have for this repo is [ROS](https://www.ros.org/). We are currently using Ubuntu 18.04, so we're on ROS Melodic.
[Here's a link](http://wiki.ros.org/melodic/Installation/Ubuntu) to the instalation page for ROS Melodic.

The other main dependency we have is a ros package called SMACH. SMACH is our state machine manager. It allows us to write easily executable code that we can switch around and change as we please.
To install, it should be a simple `sudo apt install ros-melodic-smach` 

Here's a copy from the ROS instalation page of how to install ros:

```bash
# Install ROS Melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
sudo apt update && \
sudo apt install ros-melodic-desktop-full && \
sudo rosdep init && \
rosdep update && \
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc &&\
source ~/.bashrc
```

```bash
# Install additional dependencies
sudo apt update && \
sudo apt -y install git vim cmake python-catkin-pkg-modules && \
source /opt/ros/melodic/setup.bash
```

```bash
# Install extra ros packages
sudo apt -y install ros-melodic-mavlink ros-melodic-mavros ros-melodic-mavros-msgs \
    ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy
```

```bash
# More Packages
sudo geographiclib-get-geoids minimal
```

### movement\_package

If you haven't installed **ROS** by following the above commands, do that first.

Because the movement\_package is inside a catkin\_workspace, we need to initialize it a little bit differently than the others. Just run these commands:

Setup catkin\_ws:
```bash
# Setup local repo
cd path/to/sub-utilities

```

```bash
# Initalize repo
echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
git submodule init && \
git submodule update && \
cd catkin_ws/src && \
catkin_init_workspace && \
atkin_make -j $(nproc) -C .. && \
source ~/.bashrc
```
