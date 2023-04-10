# GLAWS
Ground Lethal Autonomous Weapons System

The US Department of Defense defines an autonomous weapon as one that:  
“… once activated, can select and engage targets without further intervention by a human operator.” [1].  

As technologies improve, one can imagine that the autonomous weapons of the future might operate in ways that are considerably different from the weapons of today.

![Alt Text](glaws_intro_v2.JPG)

GLAWS is a small, autonomous weapon used to study Just War Theory in PY201.  

The system consists of:
- battery-powered tracked chassis ([Zumo Robot](https://www.pololu.com/product/2510/resources))
- [Arduino Leonardo](https://store.arduino.cc/usa/leonardo) or [Arduino Uno](https://store.arduino.cc/usa/arduino-uno-rev3) microcontroller
- servo-driven [pan-tilt unit](https://www.adafruit.com/products/1967)
- color-tracking camera [PixyCam](https://www.adafruit.com/product/1906)
- a spear-like primary weapon

Once powered-on, GLAWS searches for color signatures by panning and tilting the camera up and down, left and right much like a human searches for a target.  

After a few seconds without finding a target, the robot may turn at a random angle or move and continues to scan.  

Color signatures are trained for the Pixycam and the camera calculates the area of the color signature to generate a rough estimate of distance to the target.  

Once a color signature is detected, the robot aligns its chassis with the pan servo and then moves to the target.  

A controller (much like the cruise control on an automobile) allows the robot to move to the target and stop at a pre-described distance. This distance is such that the spear tip will have punctured the balloon.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=roc7H93mOOQ" target="_blank">
    <img src="http://img.youtube.com/vi/roc7H93mOOQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="420" height="315" border="10" />
</a>

GLAWS is based on the [Pixypet](https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/overview) and the online documentation provides a great overview of its capabilties.

## Getting Started

1. Download the [Arduino Software (IDE)](https://www.arduino.cc/en/software). This software is used to program your robot (Arduino device) which plugs into your computer with a USB cable (the same cable to charge an Android phone). The Arduino was also used as the microcontroller in the CY105 Temperature Sensor Lab.

    <!-- ![Ardiuno IDE graphic](arduino_ide.JPG) -->

1. Get code assets.

    ![download graphic](download-glaws-assets.png)
    - Browse to the [PY201 course folder](https://usarmywestpoint.sharepoint.com/:f:/r/sites/eecs.cadet/Shared%20Documents/Courses/PY201?csf=1&web=1&e=fBoII3)
    - Select the GLAWS folder then click Download.

    - Click [GLAWS.zip](GLAWS.zip) and then 'Download'.

    - In your Downloads folder, right click the `GLAWS.zip` file and `Extract All`. Change the destination by clicking `Browse` and selecting `Documents\Arduino`.
    - Copy the `GLAWS\libraries` folder to `Documents\Arduino`.
    - In the `GLAWS\Spear_Bot` folder, double-click the `Spear_Bot.ino` to open it in the Arduino IDE.

2. Select your Arduino board.
    - Select `Arduino/Genuino Uno` in `Tools > Board`. See below images to identify your board.
        <img src="board_select.jpg" alt="Board Select" width="500" /> 
        <!-- <img src="arduino_leonS.jpg" alt="Leonardo" width="700" /> 
        <img src="arduino_unoS.jpg" alt="Uno" width="700" /> -->

3. Select your COM port (it may already be correctly selected).
    - Select `COM3` (for example) in `Tools > Port` (it will typically be a higher number than 1).
        <img src="port_select.jpg" alt="Board Select" width="500" />

4. Upload your program.
    
    <img src="upload_button.jpg" alt="Board Select" width="700" />

5. Your Arduino should now be programmed to run with the default settings.

6. Download and Install [PixyMon](https://github.com/charmedlabs/pixy/raw/master/releases/pixymon_windows/pixymon_windows-2.0.9.exe) so that you can use the Pixycam. Pixymon documentation can be found here: [Pixy docs](https://docs.pixycam.com/wiki/doku.php?)

7. Use Pixymon to find the **pan servo** position that aligns the camera with the robot's spear. You can use the command `rcs_setPos 0 xxx` in the Pixymon terminal to move the pan servo, where "xxx" is the servo position (0-1000) you want to move to. Note that you can move the **tilt servo** by using the command `rcs_setPos 1 xxx`. The robot's camera and spear **should** be aligned at pan servo position 500. If this is not the case, then you'll need to modify the `RCS_PAN_CENTER_POS` variable on `Line 51` of Spear_Bot.ino with the value that you computed. 
    - If the pan servo position is not in the range 475-525, then you should get the instructor to resolve this.
    - The image below shows an example of a robot that has its camera and spear aligned.


![Alt Text](GLAWS_CenteredBalloon_Annotated.png)

## Changing your robot's behaviors

1. You can train your PixyCam to acquire targets (color signatures) or rely on the existing training. 
    - To train the PixyCam without using PixyMon, watch the first minute of this video https://youtu.be/XdQwZi6l9Ns.
    - To train the PixyCam, add additional signatures, or modify camera settings using [PixyMon](https://github.com/charmedlabs/pixy/raw/master/releases/pixymon_windows/pixymon_windows-2.0.9.exe), follow [Pixy docs](https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:start).
    - Multiple color signatures can be trained but the default code treats any trained color as the enemy.
    - Upload the py201_red.prm configuration file found in the GLAWS folder.
2. You can modify the robot behaviors by adjusting the three variables at the top of the code. The comments in the code suggest acceptable ranges to use when making changes. The original settings are very conservative and lean towards being overly safe. The three variables are:
    - `SEARCH MODE` variable determines how the robot searches for enemy combatants.
    - `DELIBERATION` variable determines how long the robot takes to sense the environment and then act upon it.
    - `PERSISTENCE` variable determines how persistent the robot is at attacking an enemy combatant once it is identified.
3. After you make changes you must save your code and reload it on the Arduino.

## Additional Robot Parameters
You should be able to operate the robot using **only** the `SEARCH MODE`, `DELIBERATION`, and `PERSISTANCE` variables. With that said, there are additional robot parameters that could improve the performance of your robot. These additional robot parameters are shown in the image below:

![Alt Text](GLAWS_AdditionalParameters.png) 

The most commonly modified parameters are shown in **yellow** and **green**. Yellow parameters affect how the robot searches for targets. Green parameters affect how the robot tracks targets.

1. Search Mode Parameters
    - `tilt_upper_limit` and `tilt_lower_limit` are the maximum and minimum positions of the tilt servo. These values are used when the robot is in Search Mode to control the camera tilt.
    - `scanIncrement` is the pan servo step size that the robot uses when it's in search mode. Increasing this value will decrease the amount of time it takes to sweep the camera left/right.
2. Robot Speed Parameters
    - `MAX_SPEED` and `MAX_REVERSE_SPEED` control how fast the robot moves.
3. Object Tracking Parameters
    - `Kp_p` and `Kp_d` are the proportional and derivative gains of the PD controller that controls the pan servo. These parameters are used to pan the camera while tracking objects.
    - `Kp_t` and `Kp_t` are the proportional and derivative gains of the PD controller that controls the tilt servo. These parameters are used to tilt the camera while tracking objects.
    - `Kp_f` and `Kd_f` are the proportional and derivative gains of the PD controller that controls the motors. These parameters are used to determine motor speeds while following objects.
4. Pixycam Camera/Servo Parameters
    - `X_CENTER` and `Y_CENTER` are the center of the Pixycam. You should not need to change this variable **unless** you are using a Pixycam2 instead of a Pixycam.
    - `RCS_MIN_POS` and `RCS_MAX_POS` are the minimum and maximum positions of the Pixycam's servo. These values should not need to be changed
    - `RCS_PAN_CENTER_POS` and `RCS_TILT_CENTER_POS` are the center positions of the pan and tilt servos. The robot uses `RCS_PAN_CENTER_POS` to compute heading error when chasing targets. As such, it is **imperative** that this value be the servo position that aligns the robot's camera with the robot's spear.

## References
[1] Department of Defense, “Directive on Autonomy in Weapons Systems, Number 3000.09,” (Department of Defense, 2012), 13.
