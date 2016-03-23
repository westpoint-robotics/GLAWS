# GLAWS
Ground Lethal Autonomous Weapons System

The US Department of Defense defines an autonomous weapon as one that, “… once activated, can select and engage targets without further intervention by a human operator.” ([1](Department of Defense, “Directive on Autonomy in Weapons Systems, Number 3000.09,” (Department of Defense, 2012), 13.)). As technologies improve, one can imagine that the autonomous weapons of the future might operate in ways that are considerably different from the weapons of today.

![Alt Text](https://github.com/westpoint-robotics/GLAWS/blob/master/glaws_intro.jpg)

GLAWS is a small, autonomous weapon used to study Just War Theory in PY201.  The system consists of a battery-powered tracked chassis ([Sumobot] (https://www.pololu.com/product/2510/resources)), an [Arduino Leonardo] (https://www.arduino.cc/en/Main/ArduinoBoardLeonardo) (microcontroller), a servo-driven pan-tilt unit, a color-tracking camera (Pixycam), and a spear-like primary weapon.  Once powered-on, GLAWS searches for color signatures by panning and tilting the camera up and down much like a human searches for a target.  After a few seconds without finding a target, the robot turns at a random angle and continues to scan. Color signatures are trained for the Pixycam and the camera calculates the area of the color to generate a rough estimate on distance. Once a color signature is detected, the robot aligns its chassis with the pan servo and then moves to the target. A controller (much like the cruise control on an automobilie) allows the robot to move to the target and stop at a pre-described distance. This distance is such that the spear tip will have punctured the balloon.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=roc7H93mOOQ" target="_blank"><img src="http://img.youtube.com/vi/roc7H93mOOQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

GLAWS is based on the [Pixypet] (https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/overview) and the online documentation provides a great overview of its capabilties.

## Getting Started
---
1. Download the [Arduino Software] (https://www.arduino.cc/en/Main/Software).  The [Windows Installer] (https://www.arduino.cc/download_handler.php?f=/arduino-1.6.8-windows.exe) will set up your computer with the software. This software is used to program your Arduino Leonardo which plugs into your computer with a micro-USB cable (the same cable to charge your Android phone). The Arduino was also used as the micrcontroller in the IT105 Temperature Sensor Lab.
 
![Alt Text](https://github.com/westpoint-robotics/GLAWS/blob/master/arduino.png)

2. Copy the Pixy and ZumoMotor libraries into your Arduino library on your computer.
 - Using Windows Explorer, navigate to this folder: `My Documents/Arduino/libraries`
 - Click this [link] (file://usmasvddkorn/Cadet-courses$/XE475/Arduino_Examples/GLAWS/libraries) to open the needed libraries.
 - Click and drag these folders into `My Documents/Arduino/libraries`
3. Copy the starter sketch over to your computer.
 - Click this [link] (file:\\usmasvddkorn\Cadet-courses$\XE475\Arduino_Examples\GLAWS\Spear_Bot) to open the Spear_Bot folder.
 - Click and drag the Spear_Bot folder onto your Desktop.
 - Double-click `Spear_Bot.ino` to open the sketch.
4. Select your Arduino board.
 - Select `Arduino Leonardo` in `Tools > Board` 
 - Refer to Step 7 in this [tutorial] (https://www.arduino.cc/en/Guide/Windows)
5. Select your COM port (it may already be correctly selected).
 - Select `COM3` in `Tools > Port` (it will typically be a higher number than 1).
 - Refer to Step 8 in this [tutorial] (https://www.arduino.cc/en/Guide/Windows)
6. Upload your program.
 - Refer to Step 9 in this [tutorial] (https://www.arduino.cc/en/Guide/Windows)

 
