# semubot_eyes
The SemuBot face on the head screen: eyes, eyebrows and mouth, driven by the brain node's state.

The source of  `mouth.png`: https://creazilla.com/nodes/1556071-robot-mouth-clipart <br/>

Prerequisites:
* `pip install pygame` <br/>

Running the node in your workspace: <br/>

1. Build your workspace:
* ```cd your_ros2_workspace``` <br/>
* ```colcon build``` <br/>

2. Run:
* ```source install/local_setup.bash``` <br/>
* ```ros2 run semubot_eyes eye_controller``` <br/>

The face background is white by default. It subscribes to
`/face/background_color` (`std_msgs/msg/String`) and accepts colors in `#RRGGBB`
form. The chest display admin menu publishes this setting; run both devices in
the same ROS 2 domain to change the face background from the chest.


## Activity expressions

The face subscribes to `/semubot/state` (latched `std_msgs/msg/String` JSON)
from `semu_brain_node`. Existing face artwork is reused; eyebrows are drawn
in Pygame and quiet mouths are derived from the original mouth image.

| Brain stage | Face expression |
| --- | --- |
| `manual`, `idle` | Idle: natural blinking and occasional glances |
| `listening`, `recording` | Listening: forward gaze and attentive eyebrows |
| `transcribing`, `thinking`, `synthesizing` | Thinking: subtle eye motion and raised eyebrow |
| `speaking` | Speaking: mouth follows local WAV playback |
| `error`, `disconnected` | Concerned eyebrows and mouth |

If the activity publisher disappears, the face shows concern after a three-second
grace period and recovers when it returns. Long unchanged stages are valid;
state age is not used as a heartbeat. Fresh error entries on
`/semubot/conversation_log` show concern for five seconds; historical errors
are ignored. Actual playback takes precedence so apologies still animate.

On Linux, local SoX `play` processes are detected through `/proc`. The WAV's
40 ms RMS envelope selects mouth frames, closing during pauses and after
playback. This approximates audio timing; it is not phoneme-level lip sync.
It also supports `sayhi` and `speak -a`. Other playback programs are not tracked.
The original nose, face layout and chest-controlled background are preserved.

Build from the workspace root with `colcon build --packages-select semubot_eyes`,
then restart the eye node. To run the tests without opening a real display or
publishing to the robot's ROS domain:

```sh
source ../../install/setup.bash
PYTHONPATH="$PWD:$PYTHONPATH" ROS_DOMAIN_ID=97 SDL_VIDEODRIVER=dummy \
SDL_AUDIODRIVER=dummy python3 -m unittest discover -s test -v
```
