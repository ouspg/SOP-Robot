# Development

## Prerequisites

* [Visual Code](https://code.visualstudio.com/)
* [Vagrant](https://www.vagrantup.com/)
* [Git](https://git-scm.com/)
* HyperV, VirtualBox or VMWare (tested on HyperV)

If you are not familiar with git, take a look at this tutorial: <https://www.tutorialspoint.com/git/index.htm>

[Fork](https://docs.github.com/en/free-pro-team@latest/github/getting-started-with-github/fork-a-repo) this repository and do your development there.

Remember to `git commit` your changes and push them to remote after
every development session.

At the end of the project, create light documentation for your ROS package in english (document code, what dependencies it requires, how it should be used, how it integrates with ROS and does it require more development to be usable) in Markdown.

## Environment setup

Vagrantfile is provided for setting up the RWOS development environment. Do not store anything important in the guest. The guest is only meant for building and testing the ROS nodes.
The vagrant syncs the repository (workspace) directory to `/workspace` directory in the guest.

To provision the guest (you may have to set the provider manually):

```pwsh
vagrant up
```

Provisioning the machine for the first time can take up to 1 hour. In the meantime, download VSCode and checkout the remote development feature mentioned below. If you are using HyperV on Windows, you may have to enable `smb direct` in Windows features to be able to mount smb shares. 

Now test that the provision succeeded, so `vagrant ssh` into the guest and run `ls`, you should be see the following output:

```console
PS C:\projects\SOP-Robot> vagrant ssh
vagrant@ubuntu2004:/workspace$ ls
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
   2. For C++ development, install C++ extension (confirm that `includePath` is set correctly)

**Note: if the guest IP address changes, you have to update the IP address in the `~/ssh./config`**

**Note: Use Python 3**

## Using GUI Apps

If you are using `Hyper-V`, you can open `Hyper-V Manager` and click `Connect...`. VirtualBox and VMWare should launch with GUI when executing `vagrant up`.

### X11 Forwarding

If you wish to use x11 forwarding, it has to be enabled on the host. This is already enabled on the guest machine. This can be done by adding `ForwardX11Trusted yes` to the `vagrant-ros` section in `~/ssh./config`. In addition, you have to install x11 server on the host machine, such as [vcxsrv](https://sourceforge.net/projects/vcxsrv) on Windows.

Confirm that `$DISPLAY` is set in the ssh session. If not, set it manually by adding `export DISPLAY=HOST_MACHINE_IP:0.0` to ~/.bashrc.

**If your host is Windows you have to download the latest (v8.1+) openssh release
from [here](https://github.com/PowerShell/Win32-OpenSSH/releases).**

## Create ROS package

1. Open VSCode remote development session
2. CD into `src/` and use `ros2 pkg create` as described [here](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * For Python, `ros2 pkg create --build-type ament_python --node-name <my_node_name> <my_package_name>`
   * For C++, `ros2 pkg create --build-type ament_cmake --node-name <my_node_name> <my_package_name>`

**Note: Package node name cannot contain hyphen (setup.py entrypoint). If it does, the package compilation fails!**
