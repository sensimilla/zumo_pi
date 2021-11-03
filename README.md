# zumo_pi
ROS node running on Raspberry Pi zero w 2 riding on the back of a Pololu Zumo Robot + Arduion Uno with extras

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

Maximum throuhput on zero w 2 is 20-25 Hz 

VL53L0X_GOOD_ACCURACY_MODE      0    Good Accuracy mode - 30 Hz \
VL53L0X_BETTER_ACCURACY_MODE    1    Better Accuracy mode - 15Hz \
VL53L0X_BEST_ACCURACY_MODE      2    Best Accuracy mode - 5Hz \
VL53L0X_LONG_RANGE_MODE         3    Longe Range mode - rubbish \
VL53L0X_HIGH_SPEED_MODE         4    High Speed mode - 50 Hz \

