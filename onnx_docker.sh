#!/bin/bash

DATA_PATH=$1

docker run -it --gpus all \
      --name onnx_test -v $PWD:/workspace/anonymizer/ \
      -v $DATA_PATH:/workspace/anonymizer/data nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04\
      /bin/bash