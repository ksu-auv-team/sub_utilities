#!/usr/bin/env python3

import tensorflow.contrib.tensorrt as trt
import sys
import os
import argparse
from tf_trt_models.detection import download_detection_model, build_detection_graph

parser = argparse.ArgumentParser(description="This script will create a trt optimized graph from a retrained network. In your retreined network folder, you should have: \
                                            frozen_inference_graph.pb, three model.ckpt.<something> files, the pipeline.config file you used, and your <label-map-name>.pbtxt")
parser.add_argument('-m', '--model', help='Name of the directory containing your retrained network. You MUST put this in /data/ as that is where this script is looking for it.')
parser.add_argument('-n', '--number', default=None, help='Model number appended to model.ckpt-')

args = parser.parse_args()

if (args.model is None):
    print("Please provide a -m flag followerd by the name of your directory in the /data folder that contains the frozen graph. EX: -m my_retrained_network")
    exit()

config_path = './data/' + args.model + '/pipeline.config'
checkpoint_path = './data/' + args.model + '/model.ckpt'
if(args.number is not None):
    checkpoint_path += '-' + args.number

print(("Building detection graph from model " + args.model + "..."))
frozen_graph, input_names, output_names = build_detection_graph(
    config=config_path,
    checkpoint=checkpoint_path,
    score_threshold=0.3,
    batch_size=1
)

# score_threshold is the score below to throw out BBs
# batch_size is 1 for the Nano for speed


print ("Creating Jetson optimized graph...")
trt_graph = trt.create_inference_graph(
    input_graph_def=frozen_graph,
    outputs=output_names,
    max_batch_size=1,
    max_workspace_size_bytes=1 << 25,
    precision_mode='FP16',
    minimum_segment_size=50
)

# make the graph a trt for Jetson optimizations
# precision mode is the most important for the Nano's architecture

print ("Saving trt optimized graph...")

with open('./data/' + args.model + '/' + args.model + '_trt_graph.pb', 'wb') as f:
    f.write(trt_graph.SerializeToString())

print ("Done! Have a great day :-)")
