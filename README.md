# Ri3D-2023
Official code from the UMN Robotics Team for the 2023 FRC season!
 
 Our subsystems include:
 1) Drivetrain: We chose to build a tank drive this year, and our drivetrain commands include driving a specified distance, turning to a specified angle using a NavX gyroscope, and best of all, autonomously aiming at an Apriltag target and driving within range of it!
 2) Grabber: We can pick up both cubes and cones with our grabber subsystem, which is controlled by a single pneumatic piston! This subsystem is simple and effective.
 3) Extender: Our "Extender" subsystem is a custom linear rail system powered by two CIM motors, which extends our grabber to score. It can be controlled both manually and autonomously using an encoder and set positions. Our extender also featiures two big pneumatic pistons for raising/lowering the entire structure.
 4) Vision: One of the coolest software features we implemented this year is Apriltag tracking! We run a Photonvision pipeline on a Raspberry Pi 3 B+ to detect apriltags, and then we can track and follow either the nearest apriltag or an apriltag with a specified ID.
 5) LED Subsystem: We also wrote code for controlling RGB LED strips via a REV Blinkin module.
 
[![Env](Gifs/RI3D.gif)](https://youtu.be/eQZTAWonZkg)

1) Install the 2023 release of WPILib here: https://github.com/wpilibsuite/allwpilib/releases
2) Install the 2023 NI FRC Game Tools here: https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#473762

Our project is created using the "Timed Robot" template/style with our code being written in Java.
