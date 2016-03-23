# GLAWS
Ground Lethal Autonomous Weapons System

The US Department of Defense defines an autonomous weapon as one that, “… once activated, can select and engage targets without further intervention by a human operator.” ([1](Department of Defense, “Directive on Autonomy in Weapons Systems, Number 3000.09,” (Department of Defense, 2012), 13.)).  As technologies improve, one can imagine that the autonomous weapons of the future might operate in ways that are considerably different from the weapons of today.

![Alt Text](https://github.com/westpoint-robotics/GLAWS/blob/master/glaws_intro.jpg)

GLAWS is a small, autonomous weapon used to study Just War Theory in PY201.  The system consists of a battery-powered tracked chassis (Sumobot), an Arduino Leonardo (microcontroller), a servo-driven pan-tilt unit, a color-tracking camera (Pixycam), and a spear-like primary weapon.  Once powered-on, GLAWS searches for color signatures by panning and tilting the camera up and down much like a human searches for a target.  After a few seconds without finding a target, the robot turns at a random angle and continues to scan. Color signatures are trained for the Pixycam and the camera calculates the area of the color to generate a rough estimate on distance. Once a color signature is detected, the robot aligns its chassis with the pan servo and then moves to the target. A controller (much like the cruise control on an automobilie) allows the robot to move to the target and stop at a pre-described distance. This distance is such that the spear tip will have punctured the balloon.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=roc7H93mOOQ" target="_blank"><img src="http://img.youtube.com/vi/roc7H93mOOQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

GLAWS is based on the [Pixypet] (https://learn.adafruit.com/pixy-pet-robot-color-vision-follower-using-pixycam/overview) and the online documentation provides a great overview of its capabilties.
