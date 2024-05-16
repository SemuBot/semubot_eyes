# semubot_eyes
The SemuBot eye movement script that gets the data feed from the ReSpeaker V2.0 mic-array

The source of  `mouth.png`: https://creazilla.com/nodes/1556071-robot-mouth-clipart <br/>

Prerequisites:
* Clone the following repo: (https://github.com/SemuBot/respeaker_ros.git) and follow instructions in README.md for mic-array and `audio_common` package setup. <br/>
* `pip install pygame` <br/>

Running the node in your workspace (publisher and subscriber nodes in separate terminals): <br/>

1. Build your workspace:
* ```cd your_ros2_workspace``` <br/>
* ```colcon build``` <br/>

2. In the first terminal run:
* ```source install/local_setup.bash``` <br/>
* ```ros2 run respeaker_ros respeaker_node``` <br/>

3. In the second terminal:
* ```source install/local_setup.bash``` <br/>
* ```ros2 run semubot_eyes eye_subscriber``` <br/>

