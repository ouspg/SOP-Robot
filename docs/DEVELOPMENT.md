# Development Environment Setup

## Prerequisites

* [Visual Studio Code](https://code.visualstudio.com/)
* [Git](https://git-scm.com/)
* [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
  * Install Extension Pack for USB passthrough support (required for servo control, cameras, etc)
* [Ubuntu](https://releases.ubuntu.com/jammy/)
   * Download Ubuntu 22.04 image to your computer

**Note: after creating the machine, you will have to enable the USB controller in VM settings manually**

If you are not familiar with git, take a look at this tutorial: <https://www.tutorialspoint.com/git/index.htm>

Clone this repository, create a new branch and do your development there.

Remember to `git commit` your changes and push them to remote after
every development session.

At the end of the project, create light documentation for your ROS package in english (document code, what dependencies it requires, how it should be used, how it integrates with ROS and does it require more development to be usable) in markdown.

## Installing virtual machine
1. Open up Virtual box and create a new virtual machine with the downloaded Ubuntu image. Once created launch the virtual machine and run the Ubuntu setup, create user account and setup root/admin password and reboot. 
2. After setup is completed open at the terminal and install git and ansible:
* `sudo apt update`
* `sudo apt install software-properties-common`
* `sudo add-apt-repository --yes --update ppa:ansible/ansible`
* `sudo apt install git ansible`
it is recommended to update packages to the newest versions from the start:
* `sudo apt update`
* `sudo apt upgrade`
* Clone this repository to your home directory:
* `git clone https://github.com/ouspg/SOP-Robot.git`
and move to the cloned directory:
* `cd SOP-Robot`
* Run the ansible install script to install everything needed for the robot:
* `ansible-playbook ansible-scripts/playbook.yml --ask-become-pass`
this step can take few hours to complete.
3. After install is completed run and test the virtual version of the robot by running the following commands while you are in SOP-Robot folder in your terminal:
* `colcon build`
* `source install/setup.bash`
* `ros2 launch robot robot.fake.launch.py`
* Open a new terminal tab on the same path for each or next commands:
* `ros2 run hand_gestures hand_gestures_node`
* `python3 client/unified_arms_client.py`
* `python3 client/hand_client_tester.py`
* You can now test the hand movements by typing commands to the
hand_client_tester like for example:
* l_hand_fist
* r_hand_fist
* l_hand_open
* r_hand_open


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
