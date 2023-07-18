This package contains a service and client for finnish text-to-speech feature. 

# Usage

## Before
Check that model.pth and config.json are located in src/tts_package/resource/ folder. These should be downloaded and installed automatically when installing environment with vagrant. Scripts located in /vagrant-scripts/bootstrap.sh.

## Dependencies

* `TTS`
* `espeak-ng`

These are included in the newest version of the vagrantfile. If these are not installed during bootstrap, they need to be installed to VM before starting the service.

Install TTS
> pip install TTS <br>

And install espeak
> apt -y install espeak

## Run TTS service
> ros2 run tts_package service


## Using the service
Service can be used by calling client with terminal, giving sentence as an argument. Note that sentence should be inside quotes and in finnish.
> ros2 run tts_package client "Hei. Tässä on lause joka syntentisoidaan puheeksi."

Service will now try to synthentise sentence into .wav file located in 'src/tts_package/resource/output.wav' which can then be played from speaker.

## Potential future improvements

* Implement feature that generated .wav file will be played automatically once when created
