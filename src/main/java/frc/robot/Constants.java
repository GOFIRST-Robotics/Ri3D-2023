// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Physical Robot Constants //
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // Convert from inches to meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Measured in meters
	public static final double TRACK_WIDTH = Units.inchesToMeters(25); // Distance between centers of right and left wheels on robot (in meters)

    // Controller Input Axes //
    public static final int USB_PORT_ID = 0; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 3;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 2;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 1;
    public static final int A_BUTTON = 2;
    public static final int B_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_AXIS = 7;
    public static final int RIGHT_TRIGGER_AXIS = 8;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;

    // Motor CAN IDs //
    public static final int EXTENDER_MOTOR_1_ID = 20;
    public static final int EXTENDER_MOTOR_2_ID = 10;

    // Motor PWM Ports //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 0;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 2;
    
    // PCM (Pneumatics Control Module) Channels //
    public static final int GRABBER_SOLENOID_ID_1 = 0;
    public static final int GRABBER_SOLENOID_ID_2 = 1;
    public static final int LOWER_THE_EXTENDER_ID_1 = 0;
    public static final int LOWER_THE_EXTENDER_ID_2 = 1;
    
    // DIO (Digital Input/Output) Channels //
    public static final int LED_PWM_ID = 4;

    // Drive Constants //
    public static final boolean DRIVE_INVERT_LEFT = false; // Note: Our controller flips the vertical joystick axes
    public static final boolean DRIVE_INVERT_RIGHT = true; // Note: Our controller flips the vertical joystick axes
    public static final double GYRO_KP = 0.007;
    public static final double TRACKED_TAG_ROATION_KP = 0.0175;
    public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3;
    public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2;
    public static final double APRILTAG_POWER_CAP = 0.75;
    public static final double BEAM_BALANACED_DRIVE_KP = 0.015;
    public static final double BEAM_BALANCED_ANGLE_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
    public static final double DRIVE_TURNING_THRESHOLD_DEGREES = 3;
    public static final int LEFT_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 
    public static final int RIGHT_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 

    // Extender Constants //
    public static final boolean EXTENDER_INVERT = true;
    public static final double EXTENDER_POWER = 0.6;
    public static final double EXTENDER_SETPOINT_INTAKE = 0;
    public static final double EXTENDER_SETPOINT_1 = 50; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_2= 100; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_3 = 150; // TODO: Find the correct value for this
    public static final double EXTENDER_SETPOINT_4 = 200; // TODO: Find the correct value for this
    public static final double EXTENDER_TOLERANCE = 20.0; // TODO: Find the correct value for this

    // Position Constants //
    public static final int EXTENDER_TIMEOUT = 0; // timeout in ms; set to zero
	public static final int EXTENDER_PIDIDX = 0; // used for cascading PID; set to zero
	public static final int EXTENDER_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 
	public static final boolean EXTENDER_SENSOR_PHASE = true;
    public static final double EXTENDER_KP = 0.03; // TODO: Tune this value!

    // Vision Constants //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12); // TODO: Change me
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5); // TODO: Change me
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18); // TODO: Change me
    public static final String USB_CAMERA_NAME = "USB_Camera-B4.09.24.1";
}