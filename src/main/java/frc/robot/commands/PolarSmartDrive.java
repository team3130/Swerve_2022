// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import sensors.Navx;

/** An example command that uses an example subsystem. */
public class PolarSmartDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;

  /**
   * Creates a new ExampleCommand.
   *
   * @param chassis The subsystem used by this command.
   */
  public PolarSmartDrive(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = RobotContainer.m_driverGamepad.getRawAxis(1);
    double x = RobotContainer.m_driverGamepad.getRawAxis(2);
    double length = Math.sqrt((x * x) + (y * y));

    if ( ((Math.abs(x-0.075) < 0.075) && ((Math.abs(y-0.075) < 0.075)))) { // this is for very small joystick movements that we ignore
    }
        else if (x == 0 && y > 0) {
        m_chassis.SpinToAngle(new double[]{90}); // exceptions at tan pi/2
      } else if (x == 0 && y < 0) {
        m_chassis.SpinToAngle(new double[]{});
      } else {
        m_chassis.SpinToAngle(Math.tan(y / x)); // defining that tan is theta angle
      }

        m_chassis.Forwardy(length); // how much to go forward


   // (Navx.getAngle() + 360) % 360 - // heading point that navx finds in better numbers 0 - 360 instead of -360 to 360

    // Theta = joystick setpoint minus the heading point
        //  Now this theta point that it ends at is the new motor heading setpoint.


            // circleFixer = (Double[] angle) -> {
    //          angle[0] = ((angle[0] % 360) + 360) % 360;
    //          angle[0] += ((angle[0] > 180) ? -360 : 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
