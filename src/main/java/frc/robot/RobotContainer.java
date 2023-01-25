// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FlipFieldOrriented;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroEverything;
import frc.robot.commands.ZeroWheels;
import frc.robot.subsystems.Chassis;


import java.util.List;
import java.util.function.Consumer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static Joystick m_driverGamepad;
  private final Chassis m_chassis = new Chassis();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driverGamepad = new Joystick(0);
    configureButtonBindings();

     m_chassis.setDefaultCommand(new TeleopDrive(m_chassis));
  }

  public static Joystick getDriverGamepad() {
    return m_driverGamepad;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new ZeroWheels(m_chassis));
    new JoystickButton(m_driverGamepad, Constants.Buttons.LST_BTN_B).whileTrue(new ZeroEverything(m_chassis));
    SmartDashboard.putData(new FlipFieldOrriented(m_chassis));
  }
  public Command getAutonCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kPhysicalMaxSpeedMetersPerSecond, Constants.kMaxAccelerationDrive)
            .setKinematics(m_chassis.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1,0),
                    new Translation2d(1,-1)
            ),
            new Pose2d(2,-1, Rotation2d.fromDegrees(180)), trajectoryConfig);
    PIDController xController = new PIDController(Constants.kPXController, 0,0);
    PIDController yController = new PIDController(Constants.kPYController, 0,0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController,
            0, 0, Constants.kThetaControllerConstraints );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            m_chassis::getPose2d,
            m_chassis.getKinematics(),
            xController,
            yController,
            thetaController,
            m_chassis::setModuleStates,
            m_chassis);

     return new SequentialCommandGroup( new InstantCommand(()->m_chassis.resetOdometry(trajectory.getInitialPose())),
             swerveControllerCommand, new InstantCommand(() -> m_chassis.stopModules()));
  }

}
