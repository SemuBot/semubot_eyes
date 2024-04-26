# semubot_eyes
The SemuBot eye movement script that gets the data feed from the ReSpeaker V2.0 mic-array

Prerequisites:
* Build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2, you need the ros2 branch. <br/>
  ```git clone -b ros2 https://github.com/ros-drivers/audio_common.git```
* Clone the following repo: (https://github.com/hcrlab/respeaker_ros)
* From the repository above copy the udev rules from `respeaker_ros/config` into `/etc/udev/rules.d/`.
* In case of an error on ROS2 Humble regarding `ament_python_install_package` refer to `https://github.com/ros2/rosidl_python/pull/187` (easy fix).

Running the node in your workspace (publisher and subscriber nodes in separate terminals):

* ```ros2 run respeaker_ros respeaker_node``` <br/>
* ```ros2 run semubot_eyes eye_subscriber``` <br/>

