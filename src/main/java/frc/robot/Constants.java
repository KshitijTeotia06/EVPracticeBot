// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // drive motors: 0, 16, 2, 3 for inf recharge game bot
    public static final int MOTOR_L1_ID = 0;
    public static final int MOTOR_L2_ID = 16;
    public static final int MOTOR_R1_ID = 2;
    public static final int MOTOR_R2_ID = 3;

    // shooter and turret
    public static final int SHOOTER1 = 14;
    public static final int SHOOTER2 = 15;
    public static final int TURRT_MOTOR_ID = 9;

    // control 
    public static final int XBOX_PORT = 1;
    public static final int DRIVE_STICK_PORT = 0; // joystick
    public static final int TURN_STICK_PORT = 2; // steering wheel
    public static final int DRIVE_AXIS = 3; // 1 for joystick, 3 for right trigger xbox controller
    public static final int TURN_AXIS = 0; // 0 for wheel, 0 for left stick x-axis xbox controller

    // climb 
    public static final int CLIMB_MOTOR_1_ID = 13;
    public static final int CLIMB_MOTOR_2_ID = 12;

    // intake
    public static final int INTAKE_MOTOR = 11;
    public static final int TRANSIT_MOTOR = 8;
    public static final int BRUSH_MOTOR = 10;
    public static final int BANNER_1 = 5;
    public static final int BANNER_2 = 7;
    public static final int BANNER_3 = 9;
}
