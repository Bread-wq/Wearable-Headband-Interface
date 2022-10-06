# Wearable-Headband-Interface
The code accompanies the submission: [HAT: Head-Worn Assistive Teleoperation of Mobile Manipulators](https://arxiv.org/abs/2209.13097).<br>
Akhil Padmanabha, Qin Wang, Daphne Han, Jashkumar Diyora, Kriti Kacker, Hamza Khalid, Liang-Jung Chen, Carmel Majidi, Zackory Erickson

This repository will contain the required software we are using for the hat interface. 
For video demos, visit https://sites.google.com/view/hat-teleop/home.

## Pre-requisities
### Hardware Architecture:
![Architecture](https://user-images.githubusercontent.com/66550924/194370025-66da0544-8f57-47f5-a286-899a2da01dfc.png).
- Hat: Follow the website [https://sites.google.com/view/hat-teleop/home](https://sites.google.com/view/hat-teleop/home/build-instructions) for hat assembly instruction.
- Stretch RE1 robot https://docs.hello-robot.com/0.2/. 
- [HDMI dongle](https://www.amazon.com/Headless-Display-Emulator-Headless-1920x1080-Generation/dp/B06XT1Z9TF/ref=asc_df_B06XT1Z9TF/?tag=hyprod-20&linkCode=df0&hvadid=309751315916&hvpos=&hvnetw=g&hvrand=1849427447759673039&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9005925&hvtargid=pla-547341237007&psc=1&tag=&ref=&adgrpid=67183599252&hvpone=&hvptwo=&hvadid=309751315916&hvpos=&hvnetw=g&hvrand=1849427447759673039&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9005925&hvtargid=pla-547341237007) plugged into the robot for remote desktop.
- Wireless earbuds with built-in microphone.
- Laptop 1: For remote control of the robot computer.
- Laptop 2: For speech recognition processing. <br>
(Note: Laptop 1 and 2 can possibly be combined into using a single computer, using Remote Desktop to control the robot computer and using terminal to run the speech recognition script. Here we call them Laptop 1 and Laptop 2 for easy reference.)

### Software Installation
#### On laptop 1:
1. Remote desktop control of the robot using either [Getscreen.me](https://getscreen.me/) or [DWService](https://www.dwservice.net/).
2. On Stretch RE1, create a new user account and log into that account.
3. Install the required dependencies: 
```pip install -r Requirements_laptop1.txt ```

#### On laptop 2:
1. Install the required dependencies: 
```pip install -r Requirements_laptop2.txt ```
2. Connect to the earbuds via bluetooth.


## Instructions on how to use the repository
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


## How to connect the Hat to Stretch RE1 via bluetooth
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
## Setup communication between robot and the laptop processing speech recognition:
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

## Instructions on how to use speech recognition to switch mode
1. To start controlling the robot: wear the hat and shake the head left and right, after a "beep" sound, say "start". 
If the speech recognition parses the word correctly, it will say "start" to confirm that is what it heard.
Hold the head still for 2 seconds to wait for the computer to say "calibrated"
The default mode will be "drive mode" when first start the robot
2. To switch to another mode, shake the head left and right, wait for a "beep" sound and say "switch to <mode>"
Modes include "drive", "arm", "wrist", "gripper"
3. After each head shake, the program will be listening to the input for 3s, so try to say it clearly once "beep" sound is heard
4. If the speech recognition fails to identify the phrase said, it will say "Repeat", and we need to repeat shaking the head and saying the instruction again

# Stop the robot
1. To stop the robot from moving temporarily, move your head to the calibrated position
2. To stop the hat from controlling the robot, shake the head and say "pause"
3. If the researcher wants to stop the experiment and stop the robot, Ctrl-C to terminate ```main.py``` and press the E-Stop button on the robot to fully stop it.
  
![EStop button: the white button shown](https://user-images.githubusercontent.com/66550924/194368128-14fd9672-23ec-4a38-b5bf-83271cb101be.png)
  
  *Emergency Stop Button*

## Saving Experiment Data
1. On line 15 in ```bluetooth.py```, fill in participant number.
2. On line 85 in ```bluetooth.py```, fill in ```user_data_path= '_____'``` with the intended directory to save data.
3. Enter 'e' in the terminal running ```keyboard.py```, the terminal running ```bluetooth.py``` should print 'Saved Data' 
4. Ctrl-C to terminate both ```bluetooth.py``` and ```sr+socket+voice.py```

