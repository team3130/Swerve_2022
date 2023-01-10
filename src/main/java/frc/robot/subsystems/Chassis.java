// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private MotorControllerGroup Spin;
  private MotorControllerGroup General;
  private TalonFX[] list;

  private int sign = 1;

  public Chassis() {
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
          list[i].configMotionAcceleration(TicksPerRevolution*0.75, 0);
      }
  }

    /**
     * Takes an angle to turn to and converts it to ticks.
     * This will be relative to the direction that the bot faces when it starts
     * In order to make it relative to where the controller is facing we would have
     * to run into a wall.
     * @param controllerAngle the angle we are turning to (passed as an array because java doesn't have references)
     *              The angle being read on the controller
     */
    public double Kuglification(double controllerAngle) {
        double output;
        // angle being read (modded to be relative)
        double motorReading = LeftFront.getSelectedSensorPosition() % TicksPerRevolution;

        // Basically converts the angle that we want to get to into ticks
        // gets rid of any possibility of a negative number by adding 360 and modding again
        output = ((TicksPerRevolution / 360d * ((controllerAngle - Navx.getAngle()) % 360)) + 360) % 360;

        // if the angle that we need to travel is +- 90 degrees OR angle that we need t
        // o travel is greater than 370 and less than 360
        double angleToTravel = Math.abs(output - motorReading);

        // if we need to travel between and 90 and 180 degrees we will switch to the backup system instead
        if (angleToTravel > NinetyDegreesInTicks && angleToTravel <= TwoSeventyDegreesInTicks) {
            sign = -1;
            // flip because we are now traveling the wheels backwards
            if (output >= HundredEightyDegreesInTicks) {
                output -= HundredEightyDegreesInTicks;
            }
            else {
                output += HundredEightyDegreesInTicks;
            }
        }
        else {
            // assert that we are going forward
            sign = 1;
        }

        // gets the number in ticks that is closest to an equivalent value of 360 in the tick group
        double level = ((int) (LeftFront.getSelectedSensorPosition() / TicksPerRevolution) * TicksPerRevolution);

        // if coming from top and passing over the break then you need to down-shift the set point
        if (motorReading < NinetyDegreesInTicks && angleToTravel > TwoSeventyDegreesInTicks) {
            level -= TicksPerRevolution;
        }

        // if coming from bottom and going up you need to up-shift the top
        else if (motorReading > TwoSeventyDegreesInTicks && angleToTravel < NinetyDegreesInTicks) {
            level += TicksPerRevolution;
        }

        // our output value
        return output + level;
    }
    
  public void SpinToAngle(double SetPoint) { // setter that is the last step to send to the motors with motion magic
      SetPoint = Kuglification(SetPoint);
      LeftFrontSpin.set(ControlMode.MotionMagic, SetPoint);
      LeftBackSpin.set(ControlMode.MotionMagic, SetPoint);
      RightFrontSpin.set(ControlMode.MotionMagic, SetPoint);
      RightBackSpin.set(ControlMode.MotionMagic, SetPoint);
  }

  public void Drive(double Angle, double Magnitude) {
      SpinToAngle(Angle);
      General.set(Magnitude * sign);
  }

  public void Forwardy(double joystick) {
      General.set(joystick * sign);
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