# Anonymizer
An anonymizer for pravicy protection of human faces and vehicle plates. This is a customized fork originally from archived project of [understand-ai](https://github.com/understand-ai/anonymizer). The original code is developed with tensorflow-1.11 which is a fairly old version with poor GPU support in recent GPUS. With some tricks, the [ONNX-RUNTIME](https://onnxruntime.ai/) provides a work around to running on fairly new GPUS (i.e. CUDA 11.*). Guidelines for ONNX-RUNTIME version is [here](#onnx-ify-anonymizer).

## Installation

### Preparation
- clone the repo
  ```shell
  git clone -b http://gitlab.ram-lab.com/ramlab_dataset_sensor/anonymizer.git
  ```
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
### Run Docker
```shell
# one shot run
docker run -it --rm -v $PWD:/workspace/anonymizer ros-noetic-miniconda:latest -v ${path_to_dataset}:/workspace/anonymizer/data ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
# keep the container
# docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v ${path_to_dataset}:/workspace/anonymizer/data  ros-noetic-miniconda:latest /bin/bash -c "cd /workspace/anonymizer /bin/bash"
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v ${path_to_dataset}:/workspace/anonymizer/data  ros-noetic-miniconda:latest /bin/bash 
# using the shell script

# server8
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v /data_shared_fast/Data/jwyu/fp-develop:/workspace/anonymizer/data  11710414/ros-noetic-miniconda:latest /bin/bash 
# server 11
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v /data_shared/Data/jwyu/FusionPortable_develop:/workspace/anonymizer/data  11710414/ros-noetic-miniconda:latest /bin/bash
```

### Install Dependencies
```shell
conda create --name anonymizer python=3.6 -y
conda activate anonymizer
cd /workspace/anonymizer
pip install --upgrade pip
pip install -r requirements.txt
```
## Running on RAMLAB's Servers @HKUST

### Server IP @UST
```shell
alias server8="ssh -p 23333 usrname@eez177.ece.ust.hk"
alias server5="ssh -p 23333 usrname@eez061.ece.ust.hk"
alias server11="ssh -p 23333 usrname@eez179.ece.ust.hk"
alias server13="ssh -p 23333 usrname@eez088.ece.ust.hk"
alias server3="ssh -p 23333 usrname@eez175.ece.ust.hk"
alias server12="ssh -p 23333 usrname@eez087.ece.ust.hk"
```

### Run on Server13
- Create docker container
```shell
# data pwd: /data_shared/Data/jjiao/dataset/FusionPortable_dataset_develop/sensor_data
cd /pwd_code/
docker run -it --name anonymizer -v $PWD:/workspace/anonymizer -v /data_shared/Data/jjiao/dataset/FusionPortable_dataset_develop/sensor_data:/workspace/anonymizer/data  11710414/ros-noetic-miniconda:latest /bin/bash 
```
- Create conda env as mentioned above
```shell
conda create --name anonymizer python=3.6 -y
conda activate anonymizer
cd /workspace/anonymizer
pip install --upgrade pip
pip install -r requirements.txt
```
- Run sequences
```shell
cd /workspace/anonymizer
python anonymizer/bin/anonymize_bag.py --input data/folder/to/bag/*.bag --output data/folder/to/bag/*.bag --weights weights/
```


## FusionPortable:v2's Experiments
First batch of processing (Evaluated seqs on Journal Paper)

```shell
Handheld: starbucks00, room00
Quadruped Robot: grass01, room00
Mini Hercules: campus00, parking00
Vehicle: campus00, highway00
```

### Notes on processing of vehicle's data

- Use the "vehicle" flag
  ```zsh
  # add vehicle flag for difference topic name and masked out area
  python anonymizer/bin/anonymize_bag.py --input data/folder/to/bag/*.bag --output data/folder/to/bag/*.bag --weights weights/ --vehicle
  ```
- For data on vehicle, in order to avoid the detection of the logo on hood, we need to mask out the vehicle's hood.
  ```python
  bbox for left: [300, 650, 1024, 768]
  bbox for right: [0, 620, 680, 768]  
  ```
- For data on vehilce/highway00 first 10s:
  ```python
    right_bbox = [0, 420, 430, 768]
    left_bbox = [0, 400, 350, 500]
  ```

### Experiments
- DONE: (All stored on server13)
  - vehicle/highway00(refined), highway01(refine required) campus00, campus01, downhill00, multilayer_parking00, street00, tunnel00
  - handheld(completed)/room00, room01 (refined), grass00, starbucks00, starbucks01, tunnel00
  - mini_hercules(completed)/campus00, campus01, parking00, parking01, parking02, parking03, transition00, transition01
  - quadupedal_robot(completed)/room00, grass00, grass01, transition00, tunnel00
- Running:
  - server13 -> idle 
  - server11 -> idle
  - server8 ->  idle


## Onnx-ify Anonymizer

### Preparation

#### Onnx-ify a Old TF model
- Dependencies:
  ```zsh
  pip install -U tf2onnx # install tf2onnx
  pip install packaging
  ```
- Export the original TF model to `.onnx`
  ```zsh
  python3 -m tf2onnx.convert --graphdef weights/face.pb --output face.onnx --inputs image_tensor:0 --outputs num_detections:0,detection_scores:0,detection_boxes:0

  python3 -m tf2onnx.convert --graphdef weights/plate.pb --output plate.onnx --inputs image_tensor:0 --outputs num_detections:0,detection_scores:0,detection_boxes:0
  ```
  If you bother to do this step you can directly download the exported onnx model from NAS@[ramlab_data/FusionPortable/sensor_data_v2/anonymizer/weights_onnx/](http://gofile.me/4jm56/GNx2Hr1lY). If you are not a registered user of ramlab's nas, you can download from the link with password `fusionportable`.

### Run
- Check out `onnx` branch
  ```zsh
  git checkout onnx
  ```
- Run in docker with CUDA11.6-CUDNN8
  ```zsh
  ./onnx_docker.sh ${/path/to/dataset}
  ```

- Install dependencies in docker:
  ```zsh
  apt update && apt install pip -y
  pip install -r requirements_onnx.txt
  ```