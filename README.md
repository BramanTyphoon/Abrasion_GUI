# Abrasion_GUI

This Python-backed GUI was developed to automate the operation of an oscillatory flow tunnel. 

## Motivation
The oscillatory flow tunnel in question, the Prototype Oscillatory U-Tube (POUT) was constructed by [James Bramante](https://github.com/BramanTyphoon) to quantify the rate at which waves erode submarine bedrock through sediment abrasion (essentially natural wet sandblasting), as occurs on coral reefs and in front of sea cliffs. The POUT was a 4-m tall, 3-m long u-shaped duct. Water levels in the duct were oscillated via two Automated Underwater Vehicle (AUV) [rotors](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/), and water level was tracked using a rapid-sample pressure sensor. This app was necessary to:

1. Maintain oscillations in the POUT for a user-supplied magnitude
2. Log oscillations in the POUT to a text file
3. Automate operation of the POUT for day-long experiments to save labor 

## How it works
### General description
Water levels in the POUT are recorded using a rapid-sample pressure sensor (ideally recording at ~10 Hz). Frequency domain analysis on this data is used to determine the frequency of water oscillations in the POUT, which should be close to the natural oscillation frequency of the water, a function of the length and volume of water in the POUT. This frequency should be determined empirically before each experiment by commanding the rotors to burst for a few seconds and then performing FFT on the pressure sensor oscillations that result over the next 20 seconds. 

Once the system frequency is known, the software communicates with the AUV rotor speed controllers, forcing them to oscillate power according to a cosine function with frequency equal to the system frequency. Once a user-input root-mean-square velocity (urms) is achieved in the duct, the software uses a simple feedback control algorithm to keep urms constant.

The GUI was designed to output text and graph representations of instantaneous velocity, water level, and temperature from the temperature sensor, but broken matplotlib functionality prevents use of the plotting tools (the backend to PyQt causes a segmentation fault periodically)

### Files
* [abrasion_gui.py](abrasion_gui.py) - Front end constructor for the app
* [Adafruit_I2C.py](Adafruit_I2C.py) - Contains the Python interface for the Raspberry Pi servo-board attachment to which the rotor speed controllers were connected
* [Adafruit_PWM_Servo_Driver.py](Adafruit_PWM_Servo_Driver.py) - Converts commands from mainwindow.py through rotor_driver.py to Pulse-Width-Modulated (PWM) signals to the rotor speed controllers
* [__init__.py](__init__.py) - Init file to get Python to recognize Abrasion_GUI as a package
* [mainwindow.py](mainwindow.py) - Central command of the application. Contains listeners (through sockets) and establishes daemons for reading and processing sensor data and issuing commands to rotors
* [mainWindow.ui](mainWindow.ui) - Contains style specifications for all of the GUI elements, in XML
* [ms5837.py](ms5837.py) - Driver for the pressure sensor-Python interface
* [rotor_driver.py](rotor_driver.py) - Class used to control the rotors
* [sensor_driver.py](sensor_driver.py) - Class used to control the pressure sensor
* [UI_mainWindow.py](UI_mainWindow.py) - Python code reading in style specifications from mainWindow.ui and building the GUI
* [required_packages.txt](required_packages.txt) - Packages required to run this project

## Built with
* [PyQt5](https://pypi.org/project/PyQt5/)
* [Adafruit/Raspberry Pi](https://www.adafruit.com/category/105)
* The Adafruit drivers use pre-Python3 string formatting (Python2.7 before the string formatting was back-ported). They need to be updated before this project becomes useable again. 
 
## Authors
* James Bramante - initial work - [BramanTyphoon](https://github.com/BramanTyphoon)

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

# TODO:
* TOP PRIORITY: Currently the app assumes hydrostatic pressure within the U-Tube. In fact, at the oscillation maxima, there exists a pressure gradient along the length of the water that currently has to be accounted for post-experiment. The velocities reported by the app are thus inaccurate. Taking the pressure gradient into account requires using the absolute water elevation above the pressure sensor before rotors are used. Be sure to record this water elevation, with the pressure sensor if nothing else, prior to experimental runs.
* TOP PRIORITY: The Adafruit drivers use very obsolete Python code. Find updated drivers
* Implement BrowseButton functionality in mainwindow.py
* Implement and test functionality allowing user to drive oscillations with periods at multiples of the system period. "targetTEdit" button
* Look at segmentation fault (occurs randomly after some time trying to plot water level using matplotlib-PyQt back-end)
* Investigate QTextBlock and QTextCursor errors. They don't affect functionality, but are concerning

