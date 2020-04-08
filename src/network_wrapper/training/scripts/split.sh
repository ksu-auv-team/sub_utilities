#!/bin/bash

#splits videos in a folder into images using ffmpeg

#PARAMETERS
# 1: input dir
# 2: output dir - default is ./output
# 3: sampling rate (frames per second to save) - default is 3

#Usage: e.g. ./split.sh in_vids out_imgs 10
mkdir -p ${2-output}

#loop through everything in the input folder
for f in $1/*
do
    #run ffmpeg to split videos
	ffmpeg -i $f -qscale:v 2 -r ${3-3} ./${2-output}/img_%04d.jpg
done