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
This README will provide an explaination of each top level directory in this repo.

### ROS dependencies for subdriver & movement\_package
Subdriver is what we use to control the high level architechture of the sub. It's what we use to control the state machine, write joystick control, publish out health data, etc. This is where you should go to work on new autonomy code changes.

The main dependency we have for this repo is [ROS](https://www.ros.org/). We are currently using Ubuntu 18.04, so we're on ROS Melodic.
[Here's a link](http://wiki.ros.org/melodic/Installation/Ubuntu) to the instalation page for ROS Melodic.

The other main dependency we have is a ros package called SMACH. SMACH is our state machine manager. It allows us to write easily executable code that we can switch around and change as we please.

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

These commands will install extra dependencies that we have for our project:

```bash
# Install additional dependencies
sudo apt update && \
sudo apt -y install git vim cmake python-catkin-pkg-modules && \
source /opt/ros/melodic/setup.bash
```

```bash
# Install extra ros packages
sudo apt -y install ros-melodic-mavlink ros-melodic-mavros ros-melodic-mavros-msgs \
    ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-smach
```

```bash
# More Packages
sudo geographiclib-get-geoids minimal
```
We also have some ROS<-->Arduino dependencies that must be installed also. First, download and install the [Arduino IDE](https://www.arduino.cc/en/guide/linux). Then, follow [this article](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup), or the below commands:

```bash
sudo apt install ros-melodic-rosserial-arduino ros-melodic-rosserial
```
The preceding installation steps created ros_lib, which must be copied into the Arduino build environment to enable Arduino programs to interact with ROS.

In the steps below, <sketchbook> is the directory where the Linux Arduino environment saves your sketches. Typically this is a directory called sketchbook or Arduino in your home directory. Alternately, you can install into a Windows Arduino environment.

Ros_lib installation instructions are different for groovy source (catkin) than for earlier (rosbuild) or binary releases. Be sure you've selected the correct build system above to see appropriate instructions - catkin for a groovy source build, rosbuild otherwise.

Note: you have to delete libraries/ros_lib in order to regenerate as its existence causes an error.
```bash
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```
If you start up the Arduino IDE now, you should be able to see, under the examples, new Arduino <--> ROS examples.

## Scripts
This repo also has nice startup scrips to aid in the useability of the submarine all located in the `scripts` directory. 

* `camera_node.py` will publish camera footage on the `front_raw_imgs` and `bottom_raw_imgs` topics from the video cameras.
* `vis.py` allows the user to listen on the `front_raw_imgs` and `bottom_raw_imgs` topics and show realtime video feed.
* `img_dir_pub.py` requries the user to input a directory of pictures. Will then publish those images over the front camera or bottom camera ros topics.
* `runsub.py` will run the startup process for all of AUV's different nodes including: roscore, movement\_package, execute\_witState, etc. This is the script you want to run if you are trying to run the full machine. 


## Submodules
Inside the `submodules` directory is where all the submodules of this repo live. Most of these are also required for making the sub work properly and you should checkout their README's to get a better understanding of what they all do.   

If you push a commit to a submodule, it will not ***Automatically*** show up here. You need to go into the `submodules/<your-updated-submodule>` directory, do a `git fetch`, then checkout the commit that you want. When you then return to the base `sub-utilities` directory, and do a `git status`, you should see it showing that you updated a submodule. You can then `git add` and `git commit` that like normal and it will update the `sub-utilities` repo. 

I can't stress enough how much a brief look through [this submodules tutorial](https://git-scm.com/book/en/v2/Git-Tools-Submodules) will help you if you are confued.

In this repo, we have a few submodules currently: 
 * **ncs-ros** - Legacy Neural Network stuff 
 * **network\_wrapper** - The current way we run a neural network 
 * **subdriver** - The state machine and autonomy code
 * **movement\_package** - Exsits in `catkin\_ws/src`. The wrapper for driving the sub with PWM commands
 * **submarine\_msgs\_srvs** - TODO!!

### ncs-ros  
This is legacy stuff that we used when using the Intel Movidius, but don't use anymore. Feel free to poke around though.

### network\_wrapper
The network wrapper repo is what is in charge of taking the images we publish out from the scripts above and running neural network inference on them. It then published back out both the new images with boxes drawn, and the actual detection information. 

To use it for inference or training, you will need tensorflow installed on your system. There is a non-GPU-enabled version that's just called tensorflow. But we *Typically* want the GPU enabled one because it runs significantly faster. [Here's a link](https://www.tensorflow.org/install/gpu) to how to install tensorflow-gpu. Just be aware that it's not as simple as doing a `pip install`, you will need to install some CUDA enabled stuff and probably update your video drivers. It's a hastle, but a requirement to use the GPU enabled stuff. One word of advice, **really make sure you've got the right versions**. 

### subdriver  
Subdriver is what we use to run the main autonomy portion of the system. All of the state machine code, autonomy behaviors, and other things related to actually doing underwater tasks lies in here. This repo also has information on how to write new states and state machines for future development.

Also, definately take a look at this repo to setup your networking configuration properly, because it's very confusing if you don't. 

### movement\_package  
If you haven't installed **ROS** by following the *ROS dependencies for subdriver* commands, do that first.

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
catkin_make -j $(nproc) -C .. && \
source ~/.bashrc
```

### submarine\_msgs\_srvs
This stands for Submarine Messages and Services. Our system uses custom ROS messages to standardize some information being passed around. Most importantly, we use it to define the 'detections' we get from our neural network. To learn more about custom ROS messaegs, check out [their documentation](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).

This repo also exists as a submodule of a few other repos to allow the user to only download the needed new information. 

## arduino
The `arduino` directory holds the `.ino` files that should be loaded onto any onboard arduinos. For right now, we only have one to read in the killswitch position. These scripts use ROS in an arduino environment utilizing the `rosserial` packages. See the dependency section above to install these. 

## catkin_ws
This is the repo's catkin workspace where we build our C++ code for talking to the pixhawk. If you don't know what a catkin workspace is, here is a brief description. 'Catkin' is the official build system of ROS and makes development in ROS easier than doing everything manually. It provides helpful commands to easily re-build, clean, and inspect your code as a whole. 

While we don't usually do any more active development in this area, it's useful to have an understanding of what this build system is. Check out the [ros wiki](http://wiki.ros.org/catkin/conceptual_overview) for more details.

## logs & saved_video
The `logs` directory is pretty straightforward. Whenever you run an instance of `runsub.py`, it creates a new time-stamped directory in the logs directory. This new folder holds a single file for each process that has been started. These should hold all the `stdout` and `stderr` for each process.

The `saved_video` directory is much the same. Whenever there is video being captured by the `camera_node.py` it will create a new time-stamped directory where it will save a percentage of the images (not actual video).

## USB Peripherals
This area describes the process for adding any new USB peripherals to the onboard computer. Let's say you want to add a new camera or some USB sensor, you need to make sure you can access it consistantly every time.

The Linux operating system has a couple of things that it does depending of what type of USB device is plugged into it, but they will all have one thing in common, the `/dev/` directory. The `/dev/` directory is where you can take a look at all of the USB devices that are plugged into your machine. Just type in `ls /dev` in a console and you can see everything taking up what Linux calls 'special or device files'. There's going to be a lot of things listed there, but we don't care about most of them. 

Most non-camera devices will be mapped to either `/dev/ttyUSB*` or `/dev/ttyACM*`, while cameras are typically mapped to `/dev/video*`. The reason I put a * after each of these is beacuse they are typically numbered, starting with 0 and incrementing by 1 with each additional device plugged in. The easiest way to identify which physical device is the one you care about is to type `ls /dev`, unplug it, then type `ls /dev` again and see what changed.  Now, **IMPORTANTLY** Linux does **not** keep these names constant. It completely depends on which device is plugged in first as the names are just pointers to those devices. 

So, if you have a script that relies on accessing the arduino instead of the pixhawk (which would probably both be mapped to `/dev/ttyACM0` or `/dev/ttyACM1`) then you **NEED** to give these devices static names. We can do this by writing [UDEV rules](https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux). The specific rule we care about it the `99-usb-serial.rules` which governs the USB names. 

To write a new rule, you just need to either create the file with the correct name in the correct location, or edit that file if it already exists. The below example will be to give an arduino a static name based on it's internal serial number. In our case, we have already identified that the arduino is currently mapped to /dev/ttyUSB1.

Type:
```bash
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}' | head -n1
```

This will spit out a serial number. Next, create or edit the following file with your favorite text editor, I'll use vim:
```bash
sudo vim /etc/udev/rules.d/99-usb-serial.rules
```

Finally, paste the following line in the file and save it, replacing VALUE_FROM_ABOVE with whatever the serial number was:
```bash
SUBSYSTEM=="tty", ATTRS{serial}=="VALUE_FROM_ABOVE", SYMLINK+="arduino_0"
```

Then, once you have unplugged and re-plugged in the arduino, you should be able to type `ls /dev/arduino_0` and see that arduino_0 is pointing -> to some /dev/ttyUSB*. Then, every time you need to reference that arduino in your code, you can access it by `/dev/arduino_0`.
