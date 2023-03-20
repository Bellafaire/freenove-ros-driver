# FREENOVE Three Wheel Driver
This package contains a ROS driver for the [FREENOVE Three-Wheeled Smart Kar Kit](https://www.amazon.com/Freenove-Three-Wheeled-Raspberry-Detailed-Ultrasonic/dp/B06W54XC9V/?_encoding=UTF8&pd_rd_w=kM38H&content-id=amzn1.sym.bc5f3394-3b4c-4031-8ac0-18107ac75816&pf_rd_p=bc5f3394-3b4c-4031-8ac0-18107ac75816&pf_rd_r=DGR5Y0YYAB2M4VJVA2FY&pd_rd_wg=iOD6B&pd_rd_r=b6111766-e055-4ae1-8896-bde9e0e2177e&ref_=pd_gw_ci_mcx_mr_hp_atf_m) (pictured below). 

![](https://m.media-amazon.com/images/W/IMAGERENDERING_521856-T1/images/I/61eL5Oqm0mL._AC_SL1500_.jpg)

Currently this package supports: 
- motor drive control 
- servo control 

In the future this package will support: 
- Ultrasonic sensor 
- LED indicator
- Buzzer 

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

## Working with Freenove Car on Network (Optional Guide)
You can use this package however you see fit, if you're not going to be using your car around a network you have free reign over then skip this section.
If you're just starting out or messing around with this package for fun then I reccomend controlling the car remotely from another computer. 
To do this, simply link your external computer to the roscore of the car. 
Connecting via network gives you the ability to view all the sensor data and commands to the car without the restrictions of something like ssh. 
Setting this up is quick and easy. 
For this example assume the car has a network IP of 10.0.0.2 and the computer I'm attempting to access it with has an IP of 10.0.0.3. 
If you don't know your computer's local address simply run ```ifconfig``` and look for the address listed next to ```inet``` on your network adapter.

### Freenove Car Network Setup 
First ssh into the frenove car: 
```
ssh ubuntu@10.0.0.2
``` 
and log in

Then enter the following commands to open roscore up to external connections: 

```
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_IP=10.0.0.2
```
This establishes that our roscore is running at the address ```http://10.0.0.2:11311``` and that this computer (the car) has the IP ```10.0.0.2```
You can now go ahead and launch the car control launch file: 

```
roslaunch freenove_three_wheel_driver three_wheel_control.launch
```

and that's it! we are now running the car interface 

### Connecting Computer Setup 

You'll have to perform these steps for **every** terminal instance you want connected to the freenove car. 
In a new terminal simply type the following commands: 
```
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_IP=10.0.0.3
```
Likewise here we're simply indicating that for any ROS data, we need to reach out to ```http://10.0.0.2:11311```, which we do from our own IP address
You're now free to run any program you want on your computer and have it talk to the car. 
You can do this to do rostopic sub/pub, rqt_graph, or launch a cool node on your computer that requires more processing power than the car has available. 
