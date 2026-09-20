# ALPS Touchpad Driver for FreeBSD (Stand‑alone Kernel Module)

This project provides a stand‑alone kernel module that adds some ALPS touchpad support to FreeBSD. It is designed to be loaded instead of the stock psm driver.

# WARNING!
This kernel module may cause a kernel panic during OS boot. Before installing and booting with this kernel module, ensure you have a working backup boot environment or bootable media.

## Installation
### Prerequisites
- FreeBSD 15+
- evdev support

### Building the Module
```sh
# Clone the repository
git clone https://github.com/ramdaron/freebsd-psm_alps.git
cd freebsd-psm_alps

# Build the module
make

# Install the module (copies psm_alps.ko to /boot/kernel/)
make install
```
### Loading the Driver
Add to /boot/loader.conf:
```
psm_load="YES"
psm_name="/boot/modules/psm_alps.ko"
```

and then reboot

### Configuration
All tunables are under `hw.psm.alps.*` and `hw.psm.alps_support` (the module re‑uses the `hw.psm` tree for compatibility).

Sysctl `hw.psm.alps_support` must be set in `/boot/loader.conf`

### License
The code is a combination of:
- Original FreeBSD psm driver (BSD license)
- Linux ALPS driver (GPL‑2.0‑only)

As a derived work, the combined module is distributed under GPL‑2.0‑only. See the file headers for details.
