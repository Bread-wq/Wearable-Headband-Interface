# Wearable-Headband-Interface
This repository will contain the required software we are using for the hat interface. 

# Pre-requisities
1. Install the following python packages using pip install: 
```numpy, pickle, socket, serial, keyboard, speech_recognition ```
2. Follow the website https://sites.google.com/view/hat-teleop/home for hat assembly instruction
3. A functional Stretch RE1 robot https://docs.hello-robot.com/0.2/
4. Two computers, one pair of wireless earbuds. 
  - One computer for remote control of the robot using either [Getscreen.me](https://getscreen.me/) or [DWService](https://www.dwservice.net/)
  - The other computer for running the speech recognition, and earbus which serve as a microphone are connected to it through bluetooth 
  
# Instructions on how to use the repository
1. Clone the repository to your Stretch RE1 robot and the other laptop
```sh
git clone https://github.com/Bread-wq/Wearable-Headband-Interface.git
```

2. Upload IMU\_Button\_bluetooth.ino to your TinyPico ESP32 

3. Obtain the MAC address of your TinyPico ESP32 ```<dev>```

4. Obtain the IP address of your Stretch RE1 robot, ```<Robot IP>``` 
```sh
ifconfig -a
```
- Go to bluetooth.py, replace the IP address on line 181 with ```<Robot IP>``` obtained.
- On the other laptop, in ```GUI/sr+socket+voice.py```, replace the server IP address on line 23 with ```<Robot IP>```

5. Similarly, get the IP address of the computer for running speech recognition, ```<Comp IP>```
- In ```GUI/sr+socket+voice.py```, replace the host IP address on line 20 with ```<Comp IP>```


# How to connect the Hat to Stretch RE1 via bluetooth
1. Install http://www.bluez.org/
2. Run
```
rfkill unblock all
bluetoothctl
```

3. Pair using ```bluetoothctl```:
```sh
power on
agent on
scan on
pair <dev>
```
Exit ```bluetoothctl``` by pressing ```ctl d```

4. Create serial device:
```sh
sudo rfcomm bind 0 <dev>
```
# Setup communication between robot and the laptop processing speech recognition:
1. On the robot, run 
```python home.py``` to home the robot before running each experiment
```sudo python keyboard.py```
Open another terminal and run 
```python bluetooth.py```
- Enter task number and trial number as prompted
- Enter 1 for speech recognition mode

2. On another laptop, ```cd GUI```
run ```python3 sr+socket+voice.py```
The laptop will connect to the robot via socket communication

# Instructions on how to use speech recognition to switch mode
1. To start controlling the robot: wear the hat and shake the head left and right, after a "beep" sound, say "start". 
If the speech recognition parses the word correctly, it will say "start" to confirm that is what it heard.
Hold the head still for 2 seconds to wait for the computer to say "calibrated"
The default mode will be "base mode" when first start the robot
2. To switch to another mode, shake the head left and right, wait for a "beep" sound and say "switch to <mode>"
Modes include "base/drive", "arm", "wrist", "gripper"
3. To stop the robot from moving, shake the head and say "stop"
4. After each head shake, the program will be listening to the input for 3s, so try to say it clearly once "beep" sound is heard
5. If the speech recognition fails to identify the phrase said, it will say "Repeat", and we need to repeat shaking the head and saying the instruction again.

# Saving Experiment Data
1. On line 85 in ```bluetooth.py```, fill in ```user_data_path= '_____'``` with the intended directory to save data.
2. Enter 'e' in the terminal running ```kayboard.py```, the terminal running ```bluetooth.py``` should print 'Saved Data' 
3. Ctrl-C to terminate both ```bluetooth.py``` and ```sr+socket+voice.py```

