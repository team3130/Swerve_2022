// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;

  private final SlewRateLimiter xLimiter, yLimiter,turningLimiter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public TeleopDrive(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);
    xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.kMaxAccelerationAngularDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = -RobotContainer.m_driverGamepad.getRawAxis(1); // left stick y-axis (y-axis is inverted)
    double x = RobotContainer.m_driverGamepad.getRawAxis(2); // left stick x-axis
    double theta = RobotContainer.m_driverGamepad.getRawAxis(4); // right stick x axis

    Translation2d sped = new Translation2d(x, y);
    x = Math.abs(x) > Constants.kDeadband ? x : 0.0;
    y = Math.abs(y) > Constants.kDeadband ? y : 0.0;
    theta = Math.abs(theta) > Constants.kDeadband ? theta : 0.0;

    x = xLimiter.calculate(x) * Constants.kPhysicalMaxSpeedMetersPerSecond;
    y = xLimiter.calculate(y) * Constants.kPhysicalMaxSpeedMetersPerSecond;
    theta = turningLimiter.calculate(theta) * Constants.kPhysicalMaxSpeedMetersPerSecond;

    SwerveModuleState[] moduleStates = m_chassis.getKinematics().toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x,y,theta,m_chassis.getRotation2d()));
    m_chassis.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
