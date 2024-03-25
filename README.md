# Anonymizer
An anonymizer for pravicy protection of human faces and vehicle plates. This is a customized fork originally from archived project of [understand-ai](https://github.com/understand-ai/anonymizer). The original code is developed with tensorflow-1.11 which is a fairly old version with poor GPU support in recent GPUs. With some tricks, the [ONNX-RUNTIME](https://onnxruntime.ai/) provides a work around to running on fairly new GPUs (i.e. CUDA 11.*). Guidelines for ONNX-RUNTIME version is [here](#onnx-ify-anonymizer).

## Installation

### Preparation
- Clone the Repo
  ```shell
  git clone https://github.com/jarvisyjw/Anonymizer.git
  ```
- Download the [pre-trained weights](https://drive.google.com/drive/folders/1cuu-lvG8Z6j8K9f66vxztk5XzaqRGI-e?usp=drive_link)
  
  ```shell
  # two .pb files
  weights_plate_v1.0.0.pb
  weights_face_v1.0.0.pb
  ```
- Build Docker Container
  ```bash
  # build from local
  docker build -t ros-noetic-miniconda:latest .
  # if build failed pull from docker hub
  docker pull 11710414/ros-noetic-miniconda:latest
  ```

### Install Dependencies
```shell
conda create --name anonymizer python=3.6 -y
conda activate anonymizer
cd /workspace/anonymizer
pip install --upgrade pip
pip install -r requirements.txt
```

### TODO:
  - [x] A docker container with ROS1 and CUDA11.6 compatibility.
  - [ ] Add instructions on easy run
  - [ ] Add figures for illustration
  - [ ] Add scripts directly process the rosbags.
  - [ ] Refactor the anonymizer in the same branch that is compatiable to plug in detector and obfuscators.