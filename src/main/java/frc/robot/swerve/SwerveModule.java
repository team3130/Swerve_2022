package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private WPI_TalonFX angley;
    private WPI_TalonFX spinny;
    private CANCoder driveEncoder;
    private PIDController turningPidController;
    private double absoluteEncoderOffset;
    private NetworkTableEntry encoderType;

    public SwerveModule(int side) {
        angley = new WPI_TalonFX(Constants.turningId[side]);
        spinny = new WPI_TalonFX(Constants.spinningId[side]);
        driveEncoder = new CANCoder(Constants.CANCoders[side]);
        encoderType = SmartDashboard.getEntry("encoder " + Constants.CANCoders[side]);
        turningPidController = new PIDController(Constants.SwerveKp, Constants.SwerveKi, Constants.SwerveKd, Constants.SwerveKf);
        angley.configFactoryDefault();
        angley.setNeutralMode(NeutralMode.Brake);
        angley.configVoltageCompSaturation(Constants.MaxAngleyVoltage);
        angley.enableVoltageCompensation(true);
        angley.configOpenloopRamp(Constants.openLoopRampRate);

        spinny.configFactoryDefault();
        spinny.setNeutralMode(NeutralMode.Brake);
        spinny.configVoltageCompSaturation(Constants.MaxAngleyVoltage);
        spinny.enableVoltageCompensation(true);
        spinny.configOpenloopRamp(Constants.openLoopRampRate);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getDrivePosition(){
        return spinny.getSelectedSensorPosition()/Constants.TicksPerRevolutionSpin;
    }

    public double getTurningPosition(){
        return angley.getSelectedSensorPosition()/Constants.TicksPerRevolutionAngle;
    }
    public double getDriveVelocity() {
        return spinny.getSelectedSensorVelocity()/Constants.TicksPerRevolutionSpin;
    }

    public double getTurningVelocity() {
        return angley.getSelectedSensorVelocity()/Constants.TicksPerRevolutionAngle;
    }
    public double getAbsolutEncoderTicks() {
        return driveEncoder.getAbsolutePosition(); //TODO convert to rads

    // It scares me that this is unused
    public double getAbsolutEncoderRad() {
        return (driveEncoder.getAbsolutePosition() % Constants.CanCoderTicksPerRevolution) / Constants.CanCoderTicksPerRevolution * Math.PI * 2; //TODO check if this is rads
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }

    public void outputToShuffleboard() {
        encoderType.setDouble(getAbsolutEncoderTicks());
    }
    public void resetEncoders(){
        angley.setSelectedSensorPosition(getAbsolutEncoderTicks());
        spinny.setSelectedSensorPosition(getAbsolutEncoderTicks());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningVelocity()));
    }

    public void stop(){
        angley.set(ControlMode.PercentOutput,0);
        spinny.set(ControlMode.PercentOutput,0);
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        spinny.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        angley.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
    }


}
