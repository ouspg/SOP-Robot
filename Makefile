.PHONY: update-overview-diagram setup build test robot robot-head robot-fake setup-udev

update-overview-diagram:
	python scripts/overview.py

setup:
	pixi run setup-runtime

build:
	pixi run build

test:
	pixi run test

robot:
	pixi run robot

robot-head:
	pixi run robot-head

robot-fake:
	pixi run robot-fake

setup-udev:
	pixi run setup-udev
