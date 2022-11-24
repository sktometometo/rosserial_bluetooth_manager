# rosserial_bluetooth_manager

This package provides a node for rosserial_bluetooth

## Prerequities

```bash
cd <your workspace>/src
wstool init
wstool set rosserial_bluetooth_manager https://github.com/sktometometo/rosserial_bluetooth_manager.git --git
wstool update
wstool merge rosserial_bluetooth_manager/rosserial_bluetooth_manager.rosinstall
wstool update
```

And add below lines to /etc/sudoers by running `sudo visudo`

```
<username of linux>    ALL=(ALL) NOPASSWD:/usr/bin/rfcomm, /bin/stty, /usr/bin/bluetoothctl
```
