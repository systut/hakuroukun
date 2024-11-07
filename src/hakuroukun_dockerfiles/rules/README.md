# Confirm udev attributes for sensors
Install udev package
```
apt install -y udev
```

Check information 
```
udevadm info --query=all --name=/dev/video0 | grep 'ID\|SUBSYSTEM'
```

After that put information into rules file (from example, 99-usb-imu.rules)
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ef", ATTRS{serial}=="TAKEBISHI_CORPORATION_TSND151_AP09181748", SYMLINK+="imu", MODE="0666"

```

```
sudo cp 99-usb-gps.rules /etc/udev/rules.d/
sudo cp 99-usb-imu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Port number will be /dev/imu
