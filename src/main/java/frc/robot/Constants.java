// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int CAN_LeftFront = 2;
    public static int CAN_LeftFrontSpin = 3;
    public static int CAN_RightFront = 4;
    public static int CAN_RightFrontSpin = 5;
    public static int CAN_LeftBack = 6;
    public static int CAN_LeftBackSpin = 7;
    public static int CAN_RightBack = 8;
    public static int CAN_RightBackSpin = 9;
    public static int GearRatio = 10;
    public static int TicksPerRevolution = 4096 * GearRatio;
    public static final int NinetyDegreesInTicks = TicksPerRevolution / 4;
    public static final int HundredEightyDegreesInTicks = TicksPerRevolution / 2;
    public static final int TwoSeventyDegreesInTicks = TicksPerRevolution / 4 * 3;
    public static double kMaxSpinnyVoltage = 5;
    public static double kMaxForwardyVoltage = 10;

    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
	public static final double trackWidth_m = 1.0;
	// the front-to-back distance between the drivetrain wheels, should be measured from center to center
	public static final double wheelBase_m = 1.0;

	public static final Translation2d[] moduleTranslations = {
		new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
	};

    public static final boolean kNavxReversed = true;

}
