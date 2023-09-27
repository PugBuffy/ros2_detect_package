# ROS2-YOLOv8s-TensorRT + docker
___

### 1. Prepare the enviroment

1. Install `CUDA` follow [_CUDA official website_](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#download-the-nvidia-cuda-toolkit).

2. Install `ROS2 humble` follow [_ROS2 humble installation_](https://docs.ros.org/en/humble/Installation.html).

3. Install `cv_bridge` library follow [_cv_bridge_](https://answers.ros.org/question/262329/how-to-install-cv_bridge/): 

4. Install python requirements.
`
$ pip install -r requirements.txt
`
___

### 2. Setup packages

Clone repository to your src directory:

`
$ git clone https://github.com/PugBuffy/ros2_detect_package.git
`

Go into the root of your workspace and build it:

`
$ colcon build
`

Source your overlay:


`
$ source install/setup.bash
`
___

### 3. Usage of the nodes

To run a usb-camera with camera node use:
`$ ros2 run detector camera
` 

To run a video file with camera node use:
`$ ros2 run detector camera --ros-args -p video_file:=<path_to_video>
`

To run detector node go into the ros-workspace and use:
`$ ros2 run detector detector
`

Run vizualizator node:
`$ ros2 run objects_vis draw_objects
`

If you want to run all the nodes use launch:
`$ ros2 launch detector yolov8s_detector.launch.py
`
Change a path to video file in yolov8s_detector.launch.py if you want to run your video.
___

### 4. Docker
Install on your host [_nvidia/docker2_](https://cpab.ru/kak-ispolzovat-graficheskij-processor-nvidia-s-kontejnerami-docker-cloudsavvy-it/) to use GPU in container.
Move _`Dockerfile`_  and _`requirements.txt`_ to the root of your workspace and start building docker container:

`$ docker build -t <container's name> .
`

After building use the next command to run:


`$ docker run -it --gpus all --device /dev/video0 <container's name> 
`

Source your overlay:

`
~# source install/setup.bash
`

In the container use commands from third chapter.
