# GLAWS
Ground Lethal Autonomous Weapons System

The US Department of Defense defines an autonomous weapon as one that:  
“… once activated, can select and engage targets without further intervention by a human operator.” [1].  

As technologies improve, one can imagine that the autonomous weapons of the future might operate in ways that are considerably different from the weapons of today.

![Alt Text](glaws_intro.jpg)

GLAWS is a small, autonomous weapon used to study Just War Theory in PY201.  

The system consists of:
- battery-powered tracked chassis ([Sumobot](https://www.pololu.com/product/2510/resources))
- [Arduino Leonardo](https://store.arduino.cc/usa/leonardo) or [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3) microcontroller.
- servo-driven [pan-tilt unit](https://www.adafruit.com/products/1967)
- color-tracking camera [Pixycam](https://www.adafruit.com/product/1906)
- a spear-like primary weapon.  
https://www.arduino.cc/en/Main/arduino-uno-rev3

Once powered-on, GLAWS searches for color signatures by panning and tilting the camera up and down, left and right much like a human searches for a target.  
After a few seconds without finding a target, the robot may turn at a random angle or move and continues to scan.  
Color signatures are trained for the Pixycam and the camera calculates the area of the color signature to generate a rough estimate of distance to the target.  
Once a color signature is detected, the robot aligns its chassis with the pan servo and then moves to the target.  
A controller (much like the cruise control on an automobilie) allows the robot to move to the target and stop at a pre-described distance. This distance is such that the spear tip will have punctured the balloon.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=roc7H93mOOQ" target="_blank">
<img src="http://img.youtube.com/vi/roc7H93mOOQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="420" height="315" border="10" /></a>

GLAWS is based on the [Pixypet](https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/overview) and the online documentation provides a great overview of its capabilties.

## Getting Started

Download the [Arduino Software](https://www.arduino.cc/en/Main/Software).  The [Windows Installer](https://www.arduino.cc/download_handler.php?f=/arduino-1.8.10-windows.exe) will set up your computer with the software. This software is used to program your Arduino which plugs into your computer with a USB cable (the same cable to charge your Android phone). The Arduino was also used as the microcontroller in the IT105 Temperature Sensor Lab.

![Alt Text](arduino_ide.JPG)

1. Copy the Pixy and ZumoMotor libraries into your Arduino library on your computer.
 - Using Windows Explorer, navigate to this folder: `~\My Documents\Arduino\libraries`
 - In another window, copy and paste this location: `2rses$\XE475\Arduino_Examples\GLAWS\libraries`
 - Click and drag `Pixy` and `ZumoMotors` into `~\My Documents\Arduino\libraries`
2. Copy the starter sketch over to your computer.
 - In Windows Explorer, copy and paste this link: `\\usmasvddkorn\Cadet-courses$\XE475\Arduino_Examples\GLAWS\Spear_Bot` to open the Spear_Bot folder.
 - Click and drag the Spear_Bot folder onto your Desktop.
 - Double-click `Spear_Bot_verC.ino` to open the sketch.
3. Select your Arduino board.
 - Select `Arduino Leonardo`  or `Arduino/Genuino Uno` in `Tools > Board`. See below images to identify your board.
 <img src="board_select.jpg" alt="Board Select" width="500" /> 
 <img src="arduino_leonS.jpg" alt="Leonardo" width="700" /> <img src="arduino_unoS.jpg" alt="Uno" width="700" />
4. Select your COM port (it may already be correctly selected).
 - Select `COM3` (for exmaple) in `Tools > Port` (it will typically be a higher number than 1).
 <img src="port_select.jpg" alt="Board Select" width="500" />
5. Upload your program.
 <img src="upload_button.jpg" alt="Board Select" width="700" />
6. Your Arduino should now be programmed to run with the default settings.

## Changing your robots behaviors

1. You can train your Pixycam to acquire targets (red balloons) or rely on the existing training. 
 - To train the Pixycam watch this video https://youtu.be/XdQwZi6l9Ns.
 - You can do this without using Pixymon. This is what he does in the first 60 seconds of the video.
 - Multiple color signatures can be trained but the default code treats any trained colors as enemy.
2. You can modify the robot behaviors by adjusting the three variables at the top of the code. The comments in the code suggest acceptable ranges to use when making changes. The original settings are very conservative and lean towards being overly safe.  
The three variables are:
  - SEARCH AGGRESSION variable determines how aggressively the robot searches for enemies.
  - DELIBERATION variable determines how long the robot takes to sense the environment and then act upon it.
  - PERSISTENCE variable determines how persistent the robot is at attacking an enemy once it is identified.
3. After you make changes you must save your code and reload it on the Arduino.


[1] Department of Defense, “Directive on Autonomy in Weapons Systems, Number 3000.09,” (Department of Defense, 2012), 13.
