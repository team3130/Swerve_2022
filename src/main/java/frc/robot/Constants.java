// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int CAN_LeftFrontSteer = 37;
    public static int CAN_LeftFrontDrive = 17;
    public static int CAN_RightFrontSteer = 14;
    public static int CAN_RightFrontDrive = 5;
    public static int CAN_LeftBackSteer = 7;
    public static int CAN_LeftBackDrive = 12;
    public static int CAN_RightBackSteer = 13;
    public static int CAN_RightBackDrive = 29;
    public static int SpinnyGearRatio = 10;
    public static int AngleyGearRatio = 10;

    public static int TicksPerRevolutionAngle = 4096 * AngleyGearRatio;
    public static double kMaxSpinnyVoltage = 5;
    public static double kMaxForwardyVoltage = 10;

    // the left-to-right distance between the drivetrain wheels, should be measured from center to center
	public static final double trackWidth_m = 0.61;
	// the front-to-back distance between the drivetrain wheels, should be measured from center to center
	public static final double wheelBase_m = 0.61;

	public static final Translation2d[] moduleTranslations = {
		new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
	};

    public static final boolean kNavxReversed = true;
    public static double wheelDiameter = Units.inchesToMeters(3.86);
    public static double TicksPerRevolutionSpin = 4096 * SpinnyGearRatio * wheelDiameter  * Math.PI;
    public static int[] turningId = new int[] {CAN_LeftFrontSteer, CAN_LeftBackSteer, CAN_RightFrontSteer, CAN_RightBackSteer};
    public static int[] spinningId = new int[] {CAN_LeftFrontDrive, CAN_LeftBackDrive, CAN_RightFrontDrive, CAN_RightBackDrive};

    public static int[] CANCoders = new int[] {9, 3, 36, 11};

    public static double SwerveKp = 3;
    public static double SwerveKi = 0;
    public static double SwerveKd = 0;
    public static double SwerveKf = 0;

    public static double MaxSpinnyVoltage = 10;
    public static double MaxAngleyVoltage = 5;

    public static double openLoopRampRate = 0.7;

    public static double kPhysicalMaxSpeedMetersPerSecond = 3;

    public static double kDeadband = 0.075;

    public static double kMaxAccelerationDrive = 3;
    public static double kMaxAccelerationAngularDrive = 3;



}
