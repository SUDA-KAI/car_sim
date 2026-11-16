# Jetson Xbox-compatible controller driver

Device observed: Flydigi Direwolf 4 receiver in Xbox-compatible mode, USB `045e:028e`. USB enumeration succeeds, but no input device exists because this Jetson kernel does not ship xpad.

Source: upstream Linux stable **v5.15.148**, unmodified:
https://github.com/gregkh/linux/blob/v5.15.148/drivers/input/joystick/xpad.c
License: GPL, retained in source.

Built against the installed headers for `5.15.148-tegra`, with the system GCC 11.4.0. Build output: `../../artifacts/xpad_build.log`. Hashes: `SHA256SUMS`.

```bash
sudo bash /home/a/ros2_ws/gz_sim/drivers/xpad/install_driver.sh
```

The installer checks the exact running kernel, installs xpad.ko under `/lib/modules/5.15.148-tegra/extra/`, updates dependency metadata and loads joydev/xpad. It creates a udev rule scoped to input devices under USB `045e:028e`, granting read/write access to the invoking user's primary group and active desktop user. It does not change access to other input devices, replace the kernel, or restart the machine. Different pre-existing module/rule files cause an error instead of being overwritten.

The signed-in user must run the command because sudo currently requires a password. Installation now verified: xpad is loaded, /dev/input/event1 and /dev/input/js0 exist with user read/write access, and real ROS Joy messages are received in domain 42.

Rebuild after a kernel update using that kernel's matching headers:

```bash
make -C /lib/modules/$(uname -r)/build M=/home/a/ros2_ws/gz_sim/drivers/xpad modules -j2
```

Do not reuse this binary with a different kernel. The installer intentionally checks the currently supported version.
