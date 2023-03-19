# freenove-ros-driver

```
#enable I2C in the config file
sudo mount /dev/mmcblk0p1 /boot 
sudo nano /boot/config.txt
#then add: 
dtparam=i2c_arm=on 
dtparam=i2c1=on
and save 
sudo nano /etc/modules
#then add 
i2c-dev 
i2c-bcm2708
and save
sudo adduser ubuntu i2c  #change 'ubuntu' to username
sudo chown :i2c /dev/i2c-1
sudo chmod g+rw /dev/i2c-1
sudo usermod -a -G i2c ubuntu #change 'ubuntu' to username
```

you may also need to install i2c-tools and confirm that the driver board is connected to addr 0x18

```
sudo apt-get install i2c-tools libi2c-dev
sudo i2cdetect -y 1
```

## ROS Packages Install
assuming you want to use the camera then you'll need the usb_cam package: 

```
sudo apt-get install ros-noetic-usb-cam
```