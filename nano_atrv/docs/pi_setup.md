# Raspberry Pi Setup

Download Pi Imager v1.9.6, flash Ubuntu Server 24.04 LTS to the SD card, do not set up wifi (it's broken). Then power up the Pi, plug in Ethernet to an internet connected router and SSH into it.

## Cut power when shutdown

Run `sudo -E rpi-eeprom-config --edit`, make sure POWER_OFF_ON_HALT is enabled (this is stored on the Pi itself):

```bash
BOOT_UART=1
POWER_OFF_ON_HALT=1
BOOT_ORDER=0xf416
```

##  Additions to `/boot/firmware/config.txt`:

```bash
# Set up UART and disable B
dtoverlay=disable-bt
dtoverlay=uart0

# Ignore lack of official PSU
usb_max_current_enable=1
```

Edit `/boot/firmware/cmdline.txt`, and delete `console=serial0, 115200` otherwise serial connected to AMA0 will cause the Pi to not boot.


## RAM Logging

Edit `/etc/systemd/journald.conf` set:

```bash
Storage=volatile
SystemMaxUse=50M
```

(extends SD card lifespan)

## Swap

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Edit `/etc/sysctl.conf` and add:

```bash
vm.swappiness=10
vm.vfs_cache_pressure=50
```

## Speed up boot

Run `sudo systemctl edit --full systemd-networkd-wait-online.service` then add the line TimeoutStartSec=5 in the [Service] section to prevent waiting 2 minutes at boot if there's no ipv6.

```bash
# Remove unused services that take 30 seconds at boot
sudo systemctl disable snapd.service snapd.seeded.service cloud-init.service
```

##  APT Setup

General cleanup and dependencies:

```bash
sudo apt update && sudo apt upgrade

# Throw unattended upgrades into the trash where it belongs before it takes apt hostage
sudo systemctl stop unattended-upgrades
sudo apt remove unattended-upgrades

# Various tools, gpio, etc.
sudo apt install curl git gpg wget build-essential tar gzip zip unzip grep sed dos2unix net-tools htop iotop nload tree python3-gpiozero net-tools -y
```

### Network Manager

Install nmcli:
```bash
sudo apt install network-manager
```

Edit `sudo nano /etc/netplan/*.yaml` add `renderer: NetworkManager` after `version: 2`, then run:
```bash
sudo netplan generate
sudo netplan apply
```

Reconnect SSH and set up persistent services:
```bash
sudo systemctl enable NetworkManager
sudo systemctl start NetworkManager
```

Edit `sudo nano /etc/netplan/*.yaml` again and set it to:
```bash
network:
  version: 2
  renderer: NetworkManager
```

Then:
```bash
sudo netplan apply
sudo reboot
```

Add access point:
```bash
sudo nmcli con add con-name "NanoATRV" type wifi ifname wlan0 mode ap ssid "NanoATRV"
sudo nmcli con modify "NanoATRV" wifi-sec.key-mgmt wpa-psk
sudo nmcli con modify "NanoATRV" wifi-sec.psk "password"
sudo nmcli con modify "NanoATRV" ipv4.addresses "10.42.0.1/24"
sudo nmcli con modify "NanoATRV" ipv4.method "shared"
sudo nmcli con up "NanoATRV"
```

Check the network setup with `nmcli con show`.

Edit `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf` and disable power saving:

```bash
[connection]
wifi.powersave = 2
```


### Install ROS 2 Jazzy

Follow the official guide [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-zenoh-cpp ros-jazzy-rmw-cyclonedds-cpp
sudo rosdep init
rosdep update
mkdir -p ~/colcon_ws/src
```

Add to `~/.bashrc`
```bash
export PROMPT_COMMAND='history -a'
alias colcon_make='colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers 2'
alias ros_restart='ros2 daemon stop; ros2 daemon start'

export ROS_DOMAIN_ID=55
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/jazzy/setup.bash
source /home/vicos/colcon_ws/install/setup.bash
```

## Setup Startup Systemd Services

### Zenoh Router

```bash
sudo cp /home/vicos/colcon_ws/src/nano_atrv/nano_atrv_bringup/systemd/atrv_zenohd.service /etc/systemd/system/
sudo systemctl enable atrv_zenohd.service
sudo systemctl start atrv_zenohd.service
```

### Robot Bringup

```bash
sudo cp /home/vicos/colcon_ws/src/nano_atrv/nano_atrv_bringup/systemd/atrv_boot.service /etc/systemd/system/
sudo systemctl enable atrv_boot.service
sudo systemctl start atrv_boot.service
```