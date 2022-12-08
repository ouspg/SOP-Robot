# Development Environment Setup

## Prerequisites

* [Visual Studio Code](https://code.visualstudio.com/)
* [Git](https://git-scm.com/)
* [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
  * Install Extension Pack for USB passthrough support (required for servo control, cameras, etc)

**Note: If you do not wish to use vagrant, you can instead run the vagrant-scripts/bootstrap.sh script on a clean Ubuntu Focal Fossa installation (you have to do minor changes to the script)**

**Note: after creating the machine, you will have to enable the USB controller in VM settings manually**

If you are not familiar with git, take a look at this tutorial: <https://www.tutorialspoint.com/git/index.htm>

[Fork](https://docs.github.com/en/free-pro-team@latest/github/getting-started-with-github/fork-a-repo) this repository and do your development there.

Remember to `git commit` your changes and push them to remote after
every development session.

At the end of the project, create light documentation for your ROS package in english (document code, what dependencies it requires, how it should be used, how it integrates with ROS and does it require more development to be usable) in markdown.

## Environment setup with vagrant

Install [Vagrant](https://www.vagrantup.com/)

Vagrantfile is provided for setting up the ROS development environment. Do not store anything important in the guest. The guest is only meant for building and testing the ROS nodes.
The vagrant syncs the repository (workspace) directory to `/workspace` directory in the guest.

To provision the guest (you may have to set the provider manually):

```pwsh
vagrant up --provider=virtualbox
```

Provisioning the machine for the first time can take up to 1 hour, because after vagrant has set up the box it will install everything listed in [bootstrap.sh](vagrant-scripts/bootstrap.sh). In the meantime, download VSCode and checkout the remote development feature mentioned below. If you are using HyperV on Windows as hypervisor, you may have to enable `smb direct` in Windows features to be able to mount smb shares. 

**Note: If you have Hyper-V activated on Windows Host and decide to use Virtualbox as a hypervisor, Virtualbox will run very slowly. More information on [virtualbox forum](https://forums.virtualbox.org/viewtopic.php?t=99390) and [here](https://www.sysprobs.com/fixed-virtualbox-vms-too-slow-on-windows-host).**

Now test that the provision succeeded, so `vagrant ssh` into the guest and run `ls`, you should be see the following output:

```console
PS C:\projects\SOP-Robot> vagrant ssh
vagrant@vagrant-ros:/workspace$ ls
2  config  docs  img  launch  Makefile  README.md  scripts  sop-robot.code-workspace  src  vagrant  Vagrantfile  vagrant-scripts
```

The easiest way to setup development environment for the ROS packages is to use the [SSH Remote Development
feature](https://code.visualstudio.com/docs/remote/ssh) in [Visual Studio Code](https://code.visualstudio.com/) and to do the development inside the guest machine using this feature.

Follow [this](https://code.visualstudio.com/docs/remote/ssh) tutorial to setup the remote development on your host machine and the instructions below to setup the remote connection to your vagrant guest machine:

1. Use `vagrant up` to bring up the guest machine
2. Use `vagrant ssh-config >> ~/.ssh/config` to export the ssh-config
   * **Confirm that the file encoding is UTF-8 without BOM, especially if you are on Windows! VSCode shows the file encoding in the bottom-right corner**
3. Open `Remote-SSH: Connect to Host` in VSCode
4. Select `ros-vagrant`
5. Select `Linux`
6. Open the `/workspace` directory
7. Install whatever extensions you want to use on the guest
   1. For Python development, follow this [tutorial](https://code.visualstudio.com/docs/languages/python)
   2. For C++ development, install C++ extension (confirm that `includePath` is set correctly). [This](https://code.visualstudio.com/docs/languages/cpp) and [this](https://code.visualstudio.com/docs/cpp/config-linux) tutorials may help.

**Note: you may have to do recursive git clone (`git submodule update --init --recursive`) and run: `rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y` after `vagrant up`, if the shared folder was not mounted correctly during `vagrant up`.**

Additionally, if you lose the files of git submodules (`dynamixel-workbench` and `dynamixel-workbench-msgs`) you can get them back with `git submodule update --init --recursive`.

### Using GUI Apps

When using vagrant virtualbox provider, the GUI should pop up when executing `vagrant up`. With virtualbox you can also just start the VM from virtualbox GUI.

**Note: if your VM GUI freezes on resize, try changing the virtual machine graphics controller in the VirtualBox settings to VMSVGA**

## Create ROS package

1. Open VSCode remote development session
2. cd into `src/` and use `ros2 pkg create` as described [here](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * For Python, `ros2 pkg create --build-type ament_python --node-name <my_node_name> <my_package_name>`
   * For C++, `ros2 pkg create --build-type ament_cmake --node-name <my_node_name> <my_package_name>`

**Note: Package node name cannot contain hyphen (setup.py entrypoint). If it does, the package compilation fails!**
