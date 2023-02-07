// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class ZeroEverything extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_subsystem;
  private Limelight m_limelight;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroEverything(Chassis chassis) {
    m_subsystem = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);

  }

  public ZeroEverything(Chassis chassis, Limelight limelight) {
    m_subsystem = chassis;
    m_limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.zeroHeading();
    m_subsystem.resetEncoders();
    if (m_limelight != null && m_limelight.getCameraPosition() !=  null) {
      m_subsystem.resetPositionTo(m_limelight.getCameraPosition().toPose2d());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
