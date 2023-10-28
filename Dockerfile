FROM ros:noetic-perception

# change shell to bash
SHELL ["/bin/bash", "-c"]

RUN apt update && \
      apt -y install wget git vim

RUN mkdir -p ~/miniconda3 && \
      wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh && \
      bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3 && \
      rm -rf ~/miniconda3/miniconda.sh && \
      ~/miniconda3/bin/conda init bash

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc


# re-compile cv_bridge using python3
# RUN apt-get update && \
#       apt-get -y install python3-pip python-catkin-tools python-catkin-tools python3-dev python3-numpy

# RUN pip3 install rospkg catkin_pkg

# RUN /bin/bash -c "source /opt/ros/melodic/setup.sh"

# RUN mkdir -p /root/catkin_ws/src && \
#     cd /root/catkin_ws/src && \
#     git clone -b noetic https://github.com/ros-perception/vision_opencv.git

# WORKDIR /root/catkin_ws
# RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/include/python3.6m; catkin config --install; catkin build" && \
#       echo "source /root/catkin_ws/install/setup.bash --extend" >> ~/.bashrc

# COPY ./requirements.txt /root/requirements.txt

# RUN  /bin/bash -c "apt update; apt -y install python-pip; pip install --upgrade pip; pip3 install -r /root/requirements.txt"

# WORKDIR /home/
