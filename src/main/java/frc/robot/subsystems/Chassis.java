// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import sensors.Navx;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

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
  private TalonFX[] list;

  private int Sign = 1;

  public Chassis() {
      LeftFront = new WPI_TalonFX(Constants.CAN_LeftFront);
      LeftFrontSpin = new WPI_TalonFX(Constants.CAN_LeftFrontSpin);
      RightFront = new WPI_TalonFX(Constants.CAN_RightFront);
      RightFrontSpin = new WPI_TalonFX(Constants.CAN_RightFrontSpin);
      LeftBack = new WPI_TalonFX(Constants.CAN_LeftBack);
      LeftBackSpin = new WPI_TalonFX(Constants.CAN_LeftBackSpin);
      RightBack = new WPI_TalonFX(Constants.CAN_RightFront);
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
          list[i].configMotionCruiseVelocity(Constants.TicksPerRevolution, 0);
          list[i].configMotionAcceleration(Constants.TicksPerRevolution*0.75, 0);
      }
  }
  public void Kuglification(double[] Angle) { // this is the second to last step, converting angles to ticks
      double Reading = LeftFront.getSelectedSensorPosition() % Constants.TicksPerRevolution;
      Angle[0] = ((Constants.TicksPerRevolution / 360d) * ((Angle[0] + Navx.getAngle() + 360) % 360) );
      if ((Math.abs(Angle[0] - (Reading)) > (Constants.TicksPerRevolution/4d)) ||
              (Math.abs(Angle[0] - (Reading)) > (Constants.TicksPerRevolution/4d * 3d)) &&
              Math.abs(Angle[0] - (Reading)) < Constants.TicksPerRevolution) {
          Sign = -1;
          if (Angle[0] >= Constants.TicksPerRevolution/2d) {
              Angle[0] = Angle[0] - Constants.TicksPerRevolution/2d;
          }
          else {
              Angle[0] = Angle[0] + Constants.TicksPerRevolution/2d;
          }
      }
          else {
              Sign = 1;
          }

      if (Reading < Constants.TicksPerRevolution/4d && Angle[0] > (Constants.TicksPerRevolution/4d * 3d)) {
          double Level = ((int)(LeftFront.getSelectedSensorPosition() / Constants.TicksPerRevolution) *Constants.TicksPerRevolution);
          Angle[0] += Level - Constants.TicksPerRevolution;
      }
 }
    
  public void SpinToAngle(double[] SetPoint) { // setter that is the last step to send to the motors with motion magic
      Kuglification(SetPoint);
      LeftFrontSpin.set(ControlMode.MotionMagic, SetPoint[0]);
      LeftBackSpin.set(ControlMode.MotionMagic, SetPoint[0]);
      RightFrontSpin.set(ControlMode.MotionMagic, SetPoint[0]);
      RightBackSpin.set(ControlMode.MotionMagic, SetPoint[0]);
  }

  public void Drive(double[] Angle, double Magnitude) {
      General.set(Magnitude * Sign);
      SpinToAngle(Angle);
  }

  public void Forwardy(double joystick) {
      General.set(joystick);
  }

  public void Spinny(double joystick) {
    Spin.set(joystick);
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