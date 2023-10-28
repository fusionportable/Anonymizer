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
  docker build -t ros-noetic-miniconda:latest .
  # if build failed pull from docker hub
  docker pull 11710414/ros-noetic-miniconda:latest
  ```
## Run Docker
```shell
# one shot run
docker run -it --rm -v $PWD:/workspace/anonymizer ros-noetic-miniconda:latest -v ${path_to_dataset}:/workspace/anonymizer/data ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
# keep the container
# docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v ${path_to_dataset}:/workspace/anonymizer/data  ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v ${path_to_dataset}:/workspace/anonymizer/data  ros-noetic-miniconda:latest /bin/bash 
# using the shell script


```
## Install Dependencies
```shell
conda create --name anonymizer python=3.6 -y
conda activate anonymizer
cd /workspace/anonymizer
pip install --upgrade pip
pip install -r requirements.txt
```

## Run on Server13
- create docker container
```shell
# data pwd: /data_shared/Data/jjiao/dataset/FusionPortable_dataset_develop/sensor_data
cd /pwd_code/
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v /data_shared/Data/jjiao/dataset/FusionPortable_dataset_develop/sensor_data:/workspace/anonymizer/data  11710414/ros-noetic-miniconda:latest /bin/bash 
```
- create conda env as mentioned 
```shell
conda create --name anonymizer python=3.6 -y
conda activate anonymizer
cd /workspace/anonymizer
pip install --upgrade pip
pip install -r requirements.txt
```
- run sequences
```shell
cd /workspace/anonymizer
python anonymizer/bin/anonymize_bag.py --input data/folder/to/bag/*.bag --output data/folder/to/bag/*.bag --weights weights/
```

## First batch of processing (Evaluated seq on Paper)
```
Handheld: starbucks00 (running on server13 now), room00
Quadruped Robot: grass01, room00
Mini Hercules: campus00, parking00
Vehicle: campus00, highway00
```



## Notes
- For data on vehicle, in order to avoid the detection of the logo on hood, we need to mask out the vehicle's hood.
  ```shell
  bbox for left: [300, 650, 1024, 768]
  bbox for right: [0, 620, 680, 768]  
  ```
