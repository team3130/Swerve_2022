package frc.robot.supportingClasses;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

import java.util.ArrayDeque;
import java.util.List;

public class RawDogTrajectory {
    Command auton_command;

    final Chassis m_chassis;

    ArrayDeque<Pose2d> path;

    RawDogTrajectory(Chassis chassis) {
        m_chassis = chassis;
    }

    public void addPoints(Pose2d... poses) {
        for (Pose2d pose : poses) {
            addPoint(pose);
        }
    }

    public void addPoint(Pose2d pose) {
        path.add(pose);
    }

    public Pose2d getPoint() {
        return path.peekFirst();
    }

    public void makePath() {
            TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.kPhysicalMaxSpeedMetersPerSecond / 2, Constants.kMaxAccelerationDrive / 4)
            .setKinematics(m_chassis.getKinematics());
   Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(new Translation2d(0.5, 0)),
            new Pose2d(1,0, new Rotation2d(Math.toRadians(45))), trajectoryConfig);
  /*  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(2, 0, new Rotation2d(Math.toRadians(90)))
    ), trajectoryConfig);*/
    /*  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0,0,new Rotation2d(0)),
            new Pose2d(1.5, 0.1, new Rotation2d(Math.toRadians(30))), new Pose2d(3, 1, new Rotation2d(Math.toRadians(90))),
            new Pose2d(2.3, 1.5, new Rotation2d(Math.toRadians(120))), new Pose2d(1, 2, new Rotation2d(Math.toRadians(180))),
            new Pose2d(1.25, 2.2, new Rotation2d(Math.toRadians(0))), new Pose2d(2.25, 2.5, new Rotation2d(Math.toRadians(30))),
            new Pose2d(3, 3.25, new Rotation2d(Math.toRadians(90))), new Pose2d(2.75, 4, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0,4.5, new Rotation2d(Math.toRadians(180)))), trajectoryConfig); */
    PIDController xController = new PIDController(Constants.kPXController, 0,0);
    PIDController yController = new PIDController(Constants.kPYController, 0,0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController,
            0, 0, Constants.kThetaControllerConstraints);
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

     new SequentialCommandGroup(new InstantCommand(()->m_chassis.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand, new InstantCommand(m_chassis::stopModules));
    }
}
