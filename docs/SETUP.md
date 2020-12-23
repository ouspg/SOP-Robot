
The VM comes with ROS melodic and [MoveIt](https://moveit.ros.org/) motion planning framework pre-installed.
It is on the shared drive @ `\\kaappi\virtuaalikoneet$\VMware\SOP20\` (size 8,57 GB).
If you want to access it remotely
follow the VPN instructions: https://www.oulu.fi/th/vpn

The drive can be mounted as follows:

```
net use Z: \\kaappi\virtuaalikoneet$
```

**UPDATE: THE VM CAN BE DOWNLOADED FROM THIS [LINK](https://bit.ly/32qr1Eg)**

VM username & password: `ros`

Clone the repository:

```sh
git clone --recurse-submodules https://github.com/ouspg/SOP-Robot.git
```

Verify that `$WORKSPACE` defined in `.bashrc`
is set to the workspace directory aka to the repository location.
Remember to `source ./devel/setup.sh` after `catkin_make`.

If you wish to do development
outside the VM. Run `copy-ros-headers.sh`,
which copies header files from the VM
to `devel` directory and put this repo into a folder that is shared between the host and the guest machines.
The vscode workspace is configured to
use these headers.
If you use python do your own setup.
