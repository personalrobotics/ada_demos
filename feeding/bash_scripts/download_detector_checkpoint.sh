#!/bin/bash

echo "load acquisition checkpoint from mthrbrn"
if [ ! -d "./checkpoint" ]; then
    mkdir -p ./checkpoint
fi
scp -r prl@mthrbrn.personalrobotics.cs.washington.edu:/mnt/hard_data/Checkpoints/acquisition_detector/squeezenet.pth ./checkpoint/
