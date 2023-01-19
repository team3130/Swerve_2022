package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SwerveModule {
    private final WPI_TalonFX m_steerMotor;
    private final WPI_TalonFX m_driveMotor;
    private final CANCoder m_absoluteEncoder;
    private final PIDController turningPidController;
    private final double absoluteEncoderOffset;

    private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve Module");
    private final GenericEntry nAbsEncoderReadingTicks;
    private final GenericEntry nAbsEncoderReadingRads;
    private final GenericEntry nRelEncoderReadingTicks;
    private final GenericEntry nRelEncoderReadingRads;

    public SwerveModule(int side) {
        m_steerMotor = new WPI_TalonFX(Constants.turningId[side]);
        m_driveMotor = new WPI_TalonFX(Constants.spinningId[side]);

        m_absoluteEncoder = new CANCoder(Constants.CANCoders[side]);

        // network stuffs
        nAbsEncoderReadingTicks = tab.add("ticks abs encoder " + Constants.CANCoders[side], 0).getEntry();
        nAbsEncoderReadingRads = tab.add("rads abs encoder " + Constants.CANCoders[side], 0).getEntry();

        nRelEncoderReadingTicks = tab.add("ticks rel encoder " + Constants.CANCoders[side], 0).getEntry();
        nRelEncoderReadingRads = tab.add("rads rel encoder " + Constants.CANCoders[side], 0).getEntry();

        turningPidController = new PIDController(Constants.SwerveKp, Constants.SwerveKi, Constants.SwerveKd);

        m_steerMotor.configFactoryDefault();
        m_steerMotor.setNeutralMode(NeutralMode.Brake);
        m_steerMotor.configVoltageCompSaturation(Constants.kMaxSteerVoltage);
        m_steerMotor.enableVoltageCompensation(true);
        m_steerMotor.configOpenloopRamp(Constants.openLoopRampRate);
        m_steerMotor.setInverted(false);

        m_driveMotor.configFactoryDefault();
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.configVoltageCompSaturation(Constants.kMaxDriveVoltage);
        m_driveMotor.enableVoltageCompensation(true);
        m_driveMotor.configOpenloopRamp(Constants.openLoopRampRate);
        m_steerMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoderOffset = Constants.kCanCoderOffsets[side];

        resetEncoders();
    }

    public double getDrivePosition(){
        return m_driveMotor.getSelectedSensorPosition() * Constants.DriveTicksToMeters;
    }

    public double getTurningPosition(){
        return m_steerMotor.getSelectedSensorPosition() * Constants.SteerTicksToRads;
    }
    public double getDriveVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * Constants.DriveTicksToMetersPerSecond;
    }

    public double getTurningVelocity() {
        return m_steerMotor.getSelectedSensorVelocity() * Constants.SteerTicksToRadsPerSecond;
    }
    public double getAbsolutEncoderTicks() {
        return m_absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset;
    }

    // It scares me that this is unused
    public double getAbsolutEncoderRad() {
        return m_absoluteEncoder.getAbsolutePosition();
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }

    public void outputToShuffleboard() {
        // nAbsEncoderReadingTicks.setDouble(getAbsolutEncoderTicks());
        nAbsEncoderReadingRads.setDouble(getAbsolutEncoderRad());

        nRelEncoderReadingTicks.setDouble(m_steerMotor.getSelectedSensorPosition());
        // nRelEncoderReadingRads.setDouble(getTurningPosition());
    }
    public void resetEncoders(){
        m_steerMotor.setSelectedSensorPosition(getAbsolutEncoderTicks());
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningVelocity()));
    }

    public void stop(){
        m_steerMotor.set(ControlMode.PercentOutput, 0);
        m_driveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
        m_steerMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);
    }


}
