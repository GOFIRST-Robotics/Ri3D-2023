// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Controller Input Axes //
    public static final int USB_PORT_ID = 0; // USB port that the controller is plugged in to
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_TRIGGER_AXIS = 7;
    public static final int RIGHT_TRIGGER_AXIS = 8;
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 3;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int X_BUTTON = 1;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;

    // Motor CAN IDs //
    public static final int EXTENDER_MOTOR_1_ID = 5; // TODO: Find the correct value for this
    public static final int EXTENDER_MOTOR_2_ID = 6; // TODO: Find the correct value for this

    // Motor PWM Ports //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 0;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 2;
    
    // PCM (Pneumatics Control Module) Channels //
    public static final int GRABBER_SOLENOID_ID = 0; // TODO: Find the correct value for this
    public static final int LOWER_THE_EXTENDER_ID = 1; // TODO: Find the correct value for this
    
    // DIO (Digital Input/Output) Channels //
    public static final int LED_PWM_ID = 4;

    // Drive Constants //
    public static final boolean DRIVE_INVERT_LEFT = false; // XBox controller flips vertical axis, changing is effort
    public static final boolean DRIVE_INVERT_RIGHT = true; // // XBox controller flips vertical axis, changing is effort
    public static final double GYRO_KP = 0.01;
    public static final double TRACKED_TAG_DRIVE_KP = 0.3;
    public static final double POWER_CAP = 0.75;
    public static final double BEAM_BALANACED_DRIVE_KP = 0.025;
    public static final double BEAM_BALANCED_ANGLE_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    
    // Grabber Constants //
    public static final double GRABBER_SPEED = 0.8;
    public static final boolean GRABBER_INVERT = false;

    // Extender Constants //
    public static final boolean EXTENDER_INVERT = false;
    public static final double EXTENDER_SPEED = 0.8;
    public static final double EXTENDER_SETPOINT_INTAKE = 0; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_1 = 100; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_2= 200; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_3 = 300; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_4= 400; // TODO: Find the correct value for this

    // Position Constants //
    public static final double CAMERA_HEIGHT_METERS = 0.2; // TODO: Change me
    public static final double TARGET_HEIGHT_METERS = 0.4; // TODO: Change me
    public static final double CAMERA_PITCH_RADIANS = 0; // TODO: Change me
    public static final int EXTENDER_TIMEOUT = 0; // timeout in ms; set to zero
	public static final int EXTENDER_PIDIDX = 0; // used for cascading PID; set to zero
    public static final int EXTENDER_ENCODER_FRAME_RATE = 10;
	public static final int EXTENDER_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 
	public static final boolean EXTENDER_SENSOR_PHASE = false;
    public static final double EXTENDER_KP = 0.01; // TODO: Tune this value!
}