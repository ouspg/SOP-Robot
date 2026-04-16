# Development Environment Setup

## Prerequisites

* [Visual Studio Code](https://code.visualstudio.com/)
* [Git](https://git-scm.com/)
* [Pixi](https://pixi.sh/) — cross-platform package manager (installs ROS 2, Python deps, and build tools)
* [VirtualBox](https://www.virtualbox.org/wiki/Downloads) (optional, for VM-based development)
  * Install Extension Pack for USB passthrough support (required for servo control, cameras, etc)
* Ubuntu 22.04 (native or VM)

If you are not familiar with git, take a look at this tutorial: <https://www.tutorialspoint.com/git/index.htm>

Clone this repository, create a new branch and do your development there.

Remember to `git commit` your changes and push them to remote after
every development session.

At the end of the project, create light documentation for your ROS package in english (document code, what dependencies it requires, how it should be used, how it integrates with ROS and does it require more development to be usable) in markdown.

## Installation with Pixi

1. Install pixi (if not already installed):
   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Clone and enter the repository:
   ```bash
   git clone --recurse-submodules https://github.com/ouspg/SOP-Robot.git
   cd SOP-Robot
   ```

3. Install all dependencies (ROS 2 Humble, Python packages, build tools):
   ```bash
   pixi install
   ```
   On WSL2, the voice chatbot launch scripts will automatically switch to the
   WSLg PulseAudio Unix socket for microphone and playback support.

4. Build all ROS 2 packages:
   ```bash
   pixi run build
   ```

5. Download models (voice chatbot LLM, STT, TTS):
   ```bash
   pixi run setup-models
   ```

6. (Optional) Download legacy TTS model for `tts_package`:
   ```bash
   pixi run download-tts-model
   ```

7. (Optional) Install Dynamixel udev rules (requires sudo, only needed on real hardware):
   ```bash
   pixi run setup-udev
   ```

## Testing the simulated robot

After installation, test the simulated robot:
```bash
pixi run ros-launch-robot-fake
```

Open new terminal tabs for each:
```bash
pixi shell
ros2 run hand_gestures hand_gestures_node
```
```bash
pixi shell
python3 client/unified_arms_client.py
```
```bash
pixi shell
python3 client/hand_client_tester.py
```

You can now test hand movements by typing commands like `l_hand_fist`, `r_hand_fist`, `l_hand_open`, `r_hand_open`.

## Legacy Installation (Ansible)

The Ansible-based installation is still available in `ansible-scripts/` for reference or VM-based setups:

1. Install ansible: `sudo apt install software-properties-common && sudo add-apt-repository --yes --update ppa:ansible/ansible && sudo apt install ansible`
2. Run: `ansible-playbook ansible-scripts/playbook.yml --ask-become-pass`


## Setup vscode remote ssh
Follow the instructions of this article I found from the last 3 steps of section 1 to the end of section 2 titled "Accessing the VM from VS Code from your host machine": [VSCode Remote Development with VirtualBox](https://medium.com/nullifying-the-null/vscode-remote-development-with-virtualbox-aecd702d7933)

## About Virtualbox setup

After the VM is created, remember to add USB device filters for the USB devices you are using with the robot in the VM settings (e.g., webcam, servo controller). Oracle VM VirtualBox Manager -> Select the created VM -> Settings -> USB -> Check the 'Enable USB Controller' box and choose USB 3.0 Controller -> Add filters for devices using the '+' button. Devices might not always automatically connect to VM even the filters are configured. You can connect them manually from the menu bar. Devices -> USB -> and you will get the list of usb devices. Just select one of them and Virtualbox attaches it to the VM. Tick appears next to the device when it is attached.

**Note: The Ubuntu in VM defaults to US keyboard layout. You can change the layout from Ubuntu's options.**

**Note: if your VM GUI freezes on resize, try changing the virtual machine graphics controller in the VirtualBox settings to VMSVGA**

**Note: If you have Hyper-V activated on Windows Host and decide to use Virtualbox as a hypervisor, Virtualbox will run very slowly. More information on [virtualbox forum](https://forums.virtualbox.org/viewtopic.php?t=99390) and [here](https://www.sysprobs.com/fixed-virtualbox-vms-too-slow-on-windows-host).**

## Create ROS package

1. Open VSCode remote development session
2. cd into `src/` and use `ros2 pkg create` as described [here](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * For Python, `ros2 pkg create --build-type ament_python --node-name <my_node_name> <my_package_name>`
   * For C++, `ros2 pkg create --build-type ament_cmake --node-name <my_node_name> <my_package_name>`

**Note: Package node name cannot contain hyphen (setup.py entrypoint). If it does, the package compilation fails!**
