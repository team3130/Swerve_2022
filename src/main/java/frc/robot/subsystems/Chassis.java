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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.Side;
import frc.robot.swerve.SwerveModule;
import sensors.Navx;


public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_odometry;

  private SwerveModulePosition[] modulePositions;
  private SwerveModule[] modules;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);


  private Navx Gyro = Navx.GetInstance();

  public Chassis(){
      this (new Pose2d(), new Rotation2d());
  }

  public Chassis(Pose2d startingPos, Rotation2d startingRotation) {

      m_kinematics = new SwerveDriveKinematics(Constants.moduleTranslations);
      modulePositions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
              new SwerveModulePosition(), new SwerveModulePosition()};

      // odometry wrapper class that has functionality for cameras that report position with latency
      m_odometry = new SwerveDrivePoseEstimator(m_kinematics, startingRotation, modulePositions, startingPos);

      chassisSpeeds = new ChassisSpeeds(0, 0, 0);

      modules = new SwerveModule[4];
      modules[Side.LEFT_FRONT] = new SwerveModule(Side.LEFT_FRONT);
      modules[Side.LEFT_BACK] = new SwerveModule(Side.LEFT_BACK);
      modules[Side.RIGHT_FRONT] = new SwerveModule(Side.RIGHT_FRONT);
      modules[Side.RIGHT_BACK] = new SwerveModule(Side.RIGHT_BACK);

      new Thread(()->{
          try {
              Thread.sleep(1000);
              zeroHeading();
          } catch (Exception e) {
          }
      }).start();




  }
  public void zeroHeading(){
      Navx.resetNavX();
  }
  public double getHeading() {
      return Math.IEEEremainder(Navx.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
      return Rotation2d.fromDegrees(getHeading());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopModules(){
      modules[Side.LEFT_FRONT].stop();
      modules[Side.LEFT_BACK].stop();
      modules[Side.RIGHT_FRONT].stop();
      modules[Side.RIGHT_BACK].stop();
  }

  public void normalizeWheelSpeeds(SwerveModuleState[] desiredStates) {
      double max = desiredStates[0].speedMetersPerSecond;
      for (int i = 1 ; i< desiredStates.length ; i++ ) {
         Math.max(desiredStates[i].speedMetersPerSecond, max);
      }
      for (int i =0 ; i<desiredStates.length ; i++) {
          desiredStates[i].speedMetersPerSecond /= max;
      }
  }

  public SwerveDriveKinematics getKinematics() {
      return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
      for (int i = 0 ; i< desiredStates.length ; i++) {
          modules[i].setDesiredState(desiredStates[i]);
      }
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}