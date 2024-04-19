# semuscreen_eyes
The SemuBot eye movement script that gets the data feed from the ReSpeaker V2.0 mic-array

Prerequisites:
* Build [audio_common](https://github.com/ros-drivers/audio_common) from source until it gets released into ROS2, you need the ros2 branch. <br/>
* Clone the following repo: (https://github.com/hcrlab/respeaker_ros)
* From the repository above copy the udev rules from `respeaker_ros/config` into `/etc/udev/rules.d/`.

Running the node in your workspace:

```ros2 run semuscreen_eyes eye_subscriber_py```

