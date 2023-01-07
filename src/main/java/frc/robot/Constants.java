// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // XBox Controller Inputs //
    public static final int USB_PORT_ID = 0; // USB port that the controller is plugged in to
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

    // MOTOR CAN IDs //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 0;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int INTAKE_MOTOR_ID = 4;
    public static final int INTAKE_EXTENDER_MASTER_MOTOR_ID = 10;
    public static final int INTAKE_EXTENDER_FOLLOWER_MOTOR_ID = 11;
    public static final int FEEDER_LEFT_MOTOR_ID = 5;
    public static final int FEEDER_RIGHT_MOTOR_ID = 6;
    
    // DIO (Digital Input/Output) Channels //
    public static final int INTAKE_FORWARD_LIMIT_ID = 0;
    public static final int INTAKE_REVERSE_LIMIT_ID = 1;

    // Drive Constants //
    public static final boolean DRIVE_INVERT_LEFT = true; // XBox controller flips vertical axis, changing is effort
    public static final boolean DRIVE_INVERT_RIGHT = false; // // XBox controller flips vertical axis, changing is effort
    public static final NeutralMode DRIVE_NEUTRAL = NeutralMode.Brake;

    // Intake Constants //
    public static final double INTAKE_SPEED = 0.8;
    public static final double INTAKE_EXTEND_SPEED = 0.3;
    public static final double INTAKE_RETRACT_SPEED = -0.3;
    public static final boolean INTAKE_INVERT = false;
    public static final NeutralMode INTAKE_NEUTRAL = NeutralMode.Brake;
    public static final boolean INTAKE_EXTENDER_MASTER_INVERT = true;
    public static final InvertType INTAKE_EXTENDER_FOLLOWER_INVERT = InvertType.OpposeMaster;

    // Feeder Constants //
    public static final boolean FEEDER_INVERT_LEFT = true;
    public static final boolean FEEDER_INVERT_RIGHT = false;
    public static final NeutralMode FEEDER_NEUTRAL_LEFT = NeutralMode.Brake;
    public static final NeutralMode FEEDER_NEUTRAL_RIGHT = NeutralMode.Brake;
    public static final double FEEDER_FORWARD_SPEED = 0.8;
    public static final double FEEDER_REVERSE_SPEED = -0.8;


    
}