// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveModule;
import sensors.Navx;

import static frc.robot.Constants.TicksPerRevolution;
import static frc.robot.Constants.NinetyDegreesInTicks;
import static frc.robot.Constants.HundredEightyDegreesInTicks;
import static frc.robot.Constants.TwoSeventyDegreesInTicks;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private WPI_TalonFX LeftFront;
  private WPI_TalonFX LeftFrontSpin;
  private WPI_TalonFX RightFront;
  private WPI_TalonFX RightFrontSpin;
  private WPI_TalonFX LeftBack;
  private WPI_TalonFX LeftBackSpin;
  private WPI_TalonFX RightBack;
  private WPI_TalonFX RightBackSpin;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_odometry;

  private SwerveModulePosition[] modulePositions;
  private SwerveModule[] modules;

  private MotorControllerGroup Spin;
  private MotorControllerGroup General;
  private TalonFX[] list;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private int sign = 1;

  public Chassis(Pose2d startingPos, Rotation2d startingRotation) {
      LeftFront = new WPI_TalonFX(Constants.CAN_LeftFront);
      LeftFrontSpin = new WPI_TalonFX(Constants.CAN_LeftFrontSpin);
      RightFront = new WPI_TalonFX(Constants.CAN_RightFront);
      RightFrontSpin = new WPI_TalonFX(Constants.CAN_RightFrontSpin);
      LeftBack = new WPI_TalonFX(Constants.CAN_LeftBack);
      LeftBackSpin = new WPI_TalonFX(Constants.CAN_LeftBackSpin);
      RightBack = new WPI_TalonFX(Constants.CAN_RightBack);
      RightBackSpin = new WPI_TalonFX(Constants.CAN_RightBackSpin);

      Spin = new MotorControllerGroup(LeftFrontSpin, RightFrontSpin, LeftBackSpin, RightBackSpin);
      General = new MotorControllerGroup(LeftFront, RightFront, LeftBack, RightBack);

      list = new TalonFX[]{LeftFront, LeftFrontSpin, RightFront, RightFrontSpin, LeftBack, LeftBackSpin, RightBack, RightBackSpin};

      for (int i = 0; i < list.length; i++) {
          list[i].configFactoryDefault();
          list[i].setNeutralMode(NeutralMode.Brake);
          list[i].configVoltageCompSaturation((i % 2) * Constants.kMaxSpinnyVoltage + (i % 2 - 1) * Constants.kMaxForwardyVoltage);
          list[i].enableVoltageCompensation(true);
          list[i].configOpenloopRamp(0.7);
          list[i].configMotionCruiseVelocity(TicksPerRevolution, 0);
          list[i].configMotionAcceleration(TicksPerRevolution * 0.75, 0);
      }

      m_kinematics = new SwerveDriveKinematics(Constants.moduleTranslations);
      modulePositions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
              new SwerveModulePosition(), new SwerveModulePosition()};

      // odometry wrapper class that has functionality for cameras that report position with latency
      m_odometry = new SwerveDrivePoseEstimator(m_kinematics, startingRotation, modulePositions, startingPos);

      chassisSpeeds = new ChassisSpeeds(0, 0, 0);




  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}