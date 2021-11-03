# zumo_pi
ROS node running on Raspberry Pi Zero W 2 riding on the back of a Pololu Zumo Robot + Arduion Uno with extras

Works with forked ros_arduino_bridge in my other repository

Some nodes (brain) run on an Ubuntu VM on my grunty PC

sudo apt-get install python3-pip python3-smbus python3-serial \
sudo pip3 install git+https://github.com/pimoroni/VL53L0X-python.git \
sudo pip3 install adafruit-circuitpython-amg88xx

compile ServoBlaster \
https://github.com/richardghirst/PiBits/tree/master/ServoBlaster

place ServoBlaster systemd service file in \
/lib/systemd/system/servoblaster.service

### TOF range sensor modes
/vl53l0x/start_ranging mode


0 Good Accuracy mode - 30 Hz \
1 Better Accuracy mode - 15Hz \
2 Best Accuracy mode - 5Hz \
3 Longe Range mode - 33 Hz bit rubbish \
4 High Speed mode - 50 Hz

Maximum throuhput on zero w 2 is 20-25 Hz
