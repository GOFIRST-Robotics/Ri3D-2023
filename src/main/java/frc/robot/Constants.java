// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Inputs
    public static final int STICK_ID = 0;

    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 5;

    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int PREV_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // MOTOR IDs
    // Drivetrain PWM 0-3
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 0; // Set to coast
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 1; // Set to coast
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2; // Set to coast
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 3; // Set to coast

    // Intake CAN 10s, PWM 4
    public static final int INTAKE_MOTOR_ID = 4; // 30 amp, set to brake
    public static final int INTAKE_EXTENDER_MASTER_MOTOR_ID = 10; // Set to brake
    public static final int INTAKE_EXTENDER_FOLLOWER_MOTOR_ID = 11; // Set to brake

    // Feeder PWM 5-6
    public static final int FEEDER_LEFT_MOTOR_ID = 5; // 30 amp, set to brake
    public static final int FEEDER_RIGHT_MOTOR_ID = 6; // 30 amp, set to brake

    // Shooter CAN 20s
    public static final int SHOOTER_MOTOR_ID = 20; // 30 amp, set to coast

    // Climber PWM 7-8
    public static final int CLIMBER_1_MOTOR_ID = 7; // Set to brake
    public static final int CLIMBER_2_MOTOR_ID = 8; // Set to brake

    // DIO Ports
    public static final int INTAKE_FORWARD_LIMIT_ID = 0;
    public static final int INTAKE_REVERSE_LIMIT_ID = 1;

    // Pneumatics IDs
    public static final int SHOOTER_SOLENOID_1 = 0;
    public static final int SHOOTER_SOLENOID_2 = 1;

    // Drive Constants
    public static final boolean DRIVE_INVERT_LEFT = true; // XBox controller flips vertical axis, changing is effort
    public static final boolean DRIVE_INVERT_RIGHT = false; // // XBox controller flips vertical axis, changing is effort

    // Intake Constants
    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_EXTEND_SPEED = 0.3;
    public static final double INTAKE_RETRACT_SPEED = -0.3;
    public static final boolean INTAKE_INVERT = false;
    public static final boolean INTAKE_EXTENDER_MASTER_INVERT = true;
    public static final InvertType INTAKE_EXTENDER_FOLLOWER_INVERT = InvertType.OpposeMaster;

    // Feeder Constants
    public static final boolean FEEDER_INVERT_LEFT = true;
    public static final boolean FEEDER_INVERT_RIGHT = false;
    public static final double FEEDER_FORWARD_SPEED = 0.8;
    public static final double FEEDER_REVERSE_SPEED = -0.8;

    // Shooter Constants
    public static final boolean SHOOTER_INVERT = true;
    public static final double SHOOTER_SPEED = 0.8;
    public static final DoubleSolenoid.Value SHOOTER_DEFAULT_POSITION = DoubleSolenoid.Value.kReverse;

    // Climber Constants
    public static final double CLIMBER_RAISE_SPEED = 0.5;
    public static final double CLIMBER_CLIMB_SPEED = -1.0;
    public static final boolean CLIMBER_1_INVERT = true;
    public static final boolean CLIMBER_2_INVERT = true;
}
