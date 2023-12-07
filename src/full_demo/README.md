### Full demo

This package contains a node that combines various parts of the robot to be used in demo situations.

Currently, the parts/packages used by this node are:

- **speech_recognizer** - to hear when people talk to the robot
- **qabot** - to find responses for the recognized speech
- **tts_package** - to speak out the response produced by qabot
- **face_tracker** - to find faces and say hi when finding one
- **Hands** - to wave to the person found by face tracking

## Usage

The demo is simply run by going to the root directory of the project and running
```console
bash launch_demo.sh
```
This will start all the required nodes and services