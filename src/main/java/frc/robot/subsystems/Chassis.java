// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private MotorControllerGroup Spin;
  private MotorControllerGroup General;
  public Chassis() {
    LeftFront = new WPI_TalonFX(Constants.CAN_LeftFront);
    LeftFrontSpin = new WPI_TalonFX(Constants.CAN_LeftFrontSpin);
    RightFront = new WPI_TalonFX(Constants.CAN_RightFront);
    RightFrontSpin = new WPI_TalonFX(Constants.CAN_RightFrontSpin);
    LeftBack = new WPI_TalonFX(Constants.CAN_LeftBack);
    LeftBackSpin = new WPI_TalonFX(Constants.CAN_LeftBackSpin);
    RightBack = new WPI_TalonFX(Constants.CAN_RightFront);
    RightBackSpin = new WPI_TalonFX(Constants.CAN_RightBackSpin);
    //Spin = new MotorControllerGroup(LeftFrontSpin, RightFrontSpin, LeftBackSpin, RightBackSpin);
    General = new MotorControllerGroup(LeftFront, RightFront, LeftBack, RightBack);
  }
  public void SpinToAngle (double SetPoint) {
    LeftFrontSpin.
  }
  public void Drive(double Angle, double Magnitude) {
    General.set(Magnitude);

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
