// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

//

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
    // drive motors: 1, 2, 3, 4 for 2022 bot
    public static final int MOTOR_L1_ID = 1;
    public static final int MOTOR_L2_ID = 2;
    public static final int MOTOR_R1_ID = 3;
    public static final int MOTOR_R2_ID = 4;

    // shooter and turret
    public static final int SHOOTER1 = 10;
    public static final int SHOOTER2 = 9;
    public static final int TURRT_MOTOR_ID = 12;
    public static final int LIMIT_LEFT = 3; //dio
    public static final int LIMIT_RIGHT = 4; //dio

    // control 
    public static final int XBOX_PORT = 1;
    public static final int DRIVE_STICK_PORT = 0; // joystick
    public static final int TURN_STICK_PORT = 2; // steering wheel
    public static final int DRIVE_AXIS = 1; // 1 for joystick, 3 for right trigger xbox controller
    public static final int TURN_AXIS = 0; // 0 for wheel, 0 for left stick x-axis xbox controller

    // climb 
    public static final int CLIMB_MOTOR_1_ID = 13;
    public static final int CLIMB_MOTOR_2_ID = 12;

    // intake
    public static final int INTAKE_MOTOR = 11;
    public static final int TRANSIT_MOTOR = 5;
    public static final int BRUSH_MOTOR = 10;
    public static final int BANNER_1 = 0; // dio
    public static final int BANNER_2 = 1; // dio
    public static final int BANNER_3 = 2; // needs to be installed
    public static final double TRACK_WIDTH = 0;
    public static final int SHIFTER_L = 0; //0
    public static final int SHIFTER_R = 1; //1
}
