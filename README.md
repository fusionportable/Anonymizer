# Installation
## Preparation
- Download the weights (from NAS:ramlab_data/FusionPortable/sensor_data_v2/anonymizer/weights/)
  ```shell
  # two .pb files
  weights_plate_v1.0.0.pb
  weights_face_v1.0.0.pb
  ```
- Build Docker Container
  ```bash
  docker build -t ros-noetic-miniconda:latest
  # if build failed pull from docker hub
  docker pull 11710414/ros-noetic-miniconda:latest
  ```
## Run Docker
```shell
# one shot run
docker run -it --rm -v $PWD:/workspace/anonymizer ros-noetic-miniconda:latest -v ${path_to_dataset}:/workspace/anonymizer/data ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
# keep the container
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v ${path_to_dataset}:/workspace/anonymizer/data  ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
# using the shell script


```
## Install Dependencies
```shell
conda create --name anonymizer python=3.6
conda activate anonymizer
cd /workspace
pip install --upgrade pip
pip install -r requirements.txt
```

