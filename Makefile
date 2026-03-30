update-overview-diagram:
	python scripts/overview.py

vup:
	vagrant up --provider=virtualbox

vsh:
	vagrant ssh

# Pixi-based targets
install:
	pixi install

build:
	pixi run build

setup-models:
	pixi run setup-models

download-tts-model:
	pixi run download-tts-model

setup-udev:
	pixi run setup-udev

ros-launch-robot:
	pixi run ros-launch-robot

ros-launch-robot-fake:
	pixi run ros-launch-robot-fake

ros-launch-voice:
	pixi run ros-launch-voice

ros-full-demo:
	pixi run ros-full-demo

vagrant-reprov:
	vagrant reload --provision
