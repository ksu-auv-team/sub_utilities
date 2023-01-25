#!/usr/bin/env python

import os

# install dependencies:
os.system('./install.sh')

# download ssd_mobilenet_v1_coco and convert it to trt format:
os.system('python3 tf_download_and_trt_model.py')

# copy coco label map to mobilenet_v1 folder and rename to label_map.txt:
os.system('cp data/coco_label_map.pbtxt data/ssd_mobilenet_v1_coco/label_map.pbtxt')
