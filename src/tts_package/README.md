This package contains service and client for finnish text-to-speech feature. Service will automatically play synthesized speech when called with wanted sentence as an argument.

# Usage

## Before
Check that model.pth and config.json are located in src/tts_package/resource/ folder. These should be downloaded and installed automatically when installing environment with vagrant. Scripts located in /vagrant-scripts/bootstrap.sh.

## Dependencies

* `TTS`
* `espeak-ng`
* `simpleaudio`

These are included in the newest version of the vagrantfile. If these are not installed during bootstrap, they need to be installed to VM before starting the service.

Install TTS
> pip install TTS <br>

And install espeak
> apt -y install espeak

## Run TTS service
> ros2 run tts_package service


## Using the service
Service can be used by calling client with terminal, giving sentences as an argument. Note that sentences should be inside quotes and in finnish.
> ros2 run tts_package client "Hei. Tässä on lause joka syntentisoidaan puheeksi."

Service will now try to synthentize sentence into .wav file located in 'src/tts_package/resource/output.wav' which will then be played automatically.

## Potential future improvements

* Implement this feature to work with potential speech-to-text and chatbot features.
