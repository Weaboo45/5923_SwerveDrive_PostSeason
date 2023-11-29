/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Amp limits
    public static int PEAK_LIMIT = 40;
    public static int ENABLE_LIMIT = 30;

    // Scissor lift motor
    public static int LIFT_MOTOR = 14;

    // MEASUREMENTS
        // Drivetrain measurements
        public static double CENTER_TO_WHEEL_X = 21/2/39.37; // 21 in. to m
        public static double CENTER_TO_WHEEL_Y = 19/2/39.37; // 19 in. to m
        public static double COUNTS_PER_REVOLUTION_MAG = 4096;
        public static double WHEEL_DIAMETER = 6;
        public static double DISTANCE_PER_PULSE = ((Math.PI * WHEEL_DIAMETER) / COUNTS_PER_REVOLUTION_MAG) / 39.37;


    // PID CONSTANTS
        // Drivetrain
        public static double ROTATE_KP = 4.167 * 0.001;
        public static double ROTATE_KI = 4 * 0.001;
        public static double ROTATE_KD = 0 * 0.001;

    // MOTOR ID PORTS
        // Drivetrain motor ID ports
        public static int FRONT_LEFT_TALON_ID = 1;
        public static int BACK_LEFT_TALON_ID = 4;
        public static int FRONT_RIGHT_TALON_ID = 3;
        public static int BACK_RIGHT_TALON_ID = 2;

    // ENCODER PORTS

    // Drivetrain multipliers
    public static double ROTATION_DEADBAND = .15;   //.25
    public static double STRAFING_DEADBAND = .15;  //.75
    public static double SPEED_DEADBAND = .15; //.3
    public static double BOOST_MULTIPLIER = .65;

    // Drivetrain logic

    // Autonomous drivetrain PID
    public static double AUTON_KP = 0;
    public static double AUTON_KI = 0;
    public static double AUTON_KD = 0;
    public static double AUTON_DISTANCE_SETPOINT = 3; // feet

    // Joystick buttons
    public static int INTAKE_GRAB_BUTTON = 1;
    public static int INTAKE_LIFT_BUTTON = 2;

    public static double BOTTOM_ARM_SPEED = 0.5;
}