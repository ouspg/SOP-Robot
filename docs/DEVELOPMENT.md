# Development Environment Setup

This workspace targets Ubuntu 22.04 with ROS 2 Humble installed by Pixi into a
project-local `.pixi/` directory. Ansible is no longer part of the install path.

## Prerequisites

* [Git](https://git-scm.com/)
* [Pixi](https://pixi.prefix.dev/latest/)
* Ubuntu 22.04, either on hardware or in a virtual machine
* `espeak` on the host for the current TTS backend
* VirtualBox Extension Pack if you use a VM and need USB passthrough

If you are using real servos or a webcam in a VM, enable USB passthrough in the
VM settings before bring-up.

## Fresh Setup

```console
sudo apt update
sudo apt install -y git curl espeak
curl -fsSL https://pixi.sh/install.sh | bash
git clone --recurse-submodules https://github.com/ouspg/SOP-Robot.git
cd SOP-Robot
pixi run setup-runtime
pixi run build
```

`pixi run setup-runtime` initializes the source submodules and downloads the
TTS model.

For real Dynamixel hardware, install the udev rule once:

```console
pixi run setup-udev
```

## Daily Workflow

Build after package, launch, or configuration changes:

```console
pixi run build
```

Run tests:

```console
pixi run test
```

Open an activated shell when you want to run ROS commands directly:

```console
pixi shell
source install/local_setup.sh
ros2 launch robot robot.fake.launch.py
```

Launch aliases:

```console
pixi run robot
pixi run robot-head
pixi run robot-fake
```

## Visual Studio Code

Open the repository after Pixi has installed the environment. If VS Code does
not pick up the interpreter automatically, select:

```text
.pixi/envs/default/bin/python
```

## Create ROS Package

Use the Pixi environment for ROS package creation:

```console
pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name my_node my_package
pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name my_cpp_node my_cpp_package
```

Package node names cannot contain hyphens because Python entry points will fail
to build.
