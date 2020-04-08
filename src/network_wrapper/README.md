# network_wrapper
Jetson Nano ML install scripts, automated optimization of robotics detection models, and easy to use debugging flags

## Motivation

Installing and setting up the new Nvidia Jetson Nano was surprisingly time consuming and unintuitive. From protobuf version conflicts, to Tensorflow versions, OpenCV recompiling with GPU, models running, models optimized, and general chaos in the ranks.

This repository is KSU-AUV's set of install tools to get the Nano up and running with a convincing and scalable demo for robot-centric uses. In particular, using detection and semantic segmentation models capable at running in real-time on a robot for $100. This gives you full control of which model to run and when. 

In the repository, you'll find a few key things:

### Quick setup

If you just want to see if your nano can download and run a stock neural network without digging into any of the nitty-gritty, run the following from the root `network_wrapper` directory:

* NOTE: you will need a USB webcam plugged in, and internet access.

```
sudo nvpmodel -m 0
sudo jetson_clocks
python3 setup.py
python3 jetson_live_object_detection -d
```

`sudo nvpmodel -m 0` and `sudo jetson_clocks` put the jetson in full power mode, you may want to add them to your `~/.bashrc` so they are run at startup. `python3 setup.py` does three things. Firstly, it runs the `install.sh` script, then it runs the `tf_download_and_trt_model.py` script which downloads a pre-trained coco network and converts it to a tensorRT graph. Finally, it copies the coco label map to the newly downloaded pre-trained model's directory in the `data` directory.  Lastly, `jetson_live_object_detection -d` will run the convereted trt network we just downloaded and put it into debug mode (won't publish ros messages). This will run a tensorflow network `ssd_mobilenet_v1_coco` on a USB based webcam. 

### Install of dependencies

Getting the right versions of Tensorflow, protobufs, etc and having everyone play well on the Jetson Nano platform was a big hassle. Hopefully these will help you.

* NOTE: if you ran the `setup.py` script, you don't need to run `./install.sh`

This can be accomplished via `./install.sh` run in the root of this repository, where all the models are going to be installed and linked.

### Download of pretrained models for real-time detection 

Scripts to automatically download pretrained Tensorflow inference graphs and checkpoints, then optimize with TensorRT (which was found as a critical must-have to even *run* on the Nano).

To download coco-pre-trained networks, you need to input the name of the network you want to download after the call to `tf_download_and_trt_model.py` or it will default to ssd_mobilenet_v1_coco. The list of all pre-trained-networks can be found [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)

* NOTE: if you ran the `setup.py` script, it already downloaded ssd_mobilenet_v1_coco to the `data` directory

```
python3 tf_download_and_trt_model.py <model-name>
cp data/coco_label_map.pbtxt data/<model-name>/label_map.pbtxt #We also need to copy and re-name the label map to the new directory
```

All of these models in the tensorflow model zoo are trained on the coco dataset, but it doesn't come with the label map for what it's actually seeing. We have to copy that over to the new folder that gets created when we download the model. Along with that, we need to re-name the file to the standard label_map.pbtxt so that when the later scripts are looking for the default name, they can find it. This is what the `cp data/coco_label_map.pbtxt data/<model-name>/label_map.pbtxt` command is doing, just copying and re-naming. If you were training your own data, you would have made this file yourself. 

## Creating TRT from retrained local graph

`create_trt_from_local.py` is the script you want to run if you have already trained and frozen a tensorflow network and are trying to run it on the jetson nano. It requires your `frozen_inference_graph`, `pipeline.config`, the three `model.ckpt.*`, and your `label_map.pbtxt` to be inside a single directory in the `data` directory For your convience, an example `example_layout` directory has been created with empty (but correctly named) files inside to guide you as to how it should look and how you would call it.  The `example_layout_trt_graph.pb` is a file that will get generated in your directorry when you convert your model to a trt model.

let's say you had your frozen graph, pipeline config file, your checkpoint files, and label map all inside a directory named `example_layout` after you're done training and you have put it in the `data` directory. This is how you would go about converting it into a tensorRT optimized version:

```
#from the root 'jetson_nano_inference' directory
python3 create_trt_from_local.py -m example_layout
```

This scans the `data` directory for `example_layout` and finds the `frozen_inference_graph`, `pipeline.config`, the three `model.ckpt.*` and turns them into a single `example_layout_trt_graph.pb` file in the same directory that will be run by the jetson nano.

## Running Inference

`jetson_live_object_detection.py` is the main live object detection program. it has lots of commands that you can send it if you run it with a `--help` flag. 

If you wanted to run the fake `example_layout` trained model (which will not work) on a single USB webcam, you would do the following:
```
python3 jetson_live_object_detection.py -m example_layout # Where the -m tag stands for model
```
If you had multple cameras pluggd in, you can specify which to use:

```
python3 jetson_live_object_detection.py -m example_layout -c /dev/video2
```

If you wanted to run it in debug mode, which will not publish on ROS topics, you would run the following: 
```
python3 jetson_live_object_detection.py -m example_layout -c /dev/video2 -d # -d for debug
```

Now, let's say you wanted to do all the previous, and also wanted to change the default threshold to 70% confidence:
```
python3 jetson_live_object_detection.py -m example_layout -c /dev/video2 -d --thresh 0.7
```

If you don't have a webcam, there are both the --testVideo and --testPicture for using static testing data.    
These output a file that is the same as your test file appended with \'_output\'
```
python3 jetson_live_object_detection.py -m example_layout --testVideo example_video.avi # or:
python3 jetson_live_object_detection.py -m example_layout --testPicture example_picture.jpeg
```

you can string together as many (or as few) of these commands as you want, and you can see a full list of commands with --help

## catkin_ws
```bash
cd /path/to/jetson_nano_inference
echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd catkin_ws
catkin_make
source ~/.bashrc
```

## CV_Bridge Stuff
First, delete the devel and build folder in catkin_ws
```bash
sudo pip install -U catkin_tools
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.5m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so
catkin config --install
git clone https://github.com/ros-perception/vision_opencv.git src/vision_opencv
apt-cache show ros-melodic-cv-bridge | grep Version
cd src/vision_opencv/
git checkout <version-from-before>
cd ../../
catkin build cv_bridge
source install/setup.bash --extend
```

