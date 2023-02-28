package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.ModuleState;
import frc.lib.util.SwerveTypeConstants;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class SwerveModule {
    public int moduleNum;
    private SwerveTypeConstants swerveTypeConstants;
    private Rotation2d angleOffSet;
    private Rotation2d lastAngle;
    private TalonFX driveFalcon;
    private CANSparkMax angleNEO;
    private RelativeEncoder angleMotorEncoder;
    private CANCoder angleCANCoder;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(SWERVE_DRIVE_KS,SWERVE_DRIVE_KV,SWERVE_DRIVE_KA);

    public SwerveModule(
            int moduleNum,
            SwerveTypeConstants swerveTypeConstants,
            int driveMotorID, int angleMotorID, int canCoderID,
            Rotation2d angleOffSet){
        this.swerveTypeConstants = swerveTypeConstants;
        this.moduleNum = moduleNum;
        this.angleOffSet = angleOffSet;

        angleCANCoder = new CANCoder(canCoderID, SWERVE_CANBUS_TYPE);
        configAngleCanCoder();

        driveFalcon = new TalonFX(driveMotorID, SWERVE_CANBUS_TYPE);
        configDriveMotor();

        angleNEO = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        angleMotorEncoder = angleNEO.getEncoder();
        configAngleMotor();

        lastAngle = getState().angle;
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SWERVE_MAX_SPEED;
            driveFalcon.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio);
            driveFalcon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SWERVE_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleNEO.getPIDController().setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(angleMotorEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleCANCoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffSet.getDegrees();
        angleMotorEncoder.setPosition(absolutePosition);
    }


    private void configDriveMotor(){
        //Set drive motor config.
        TalonFXConfiguration driveFalconConfiguration = new TalonFXConfiguration();
        driveFalconConfiguration.slot0.kP = SWERVE_DRIVE_MOTOR_KP;
        driveFalconConfiguration.slot0.kI = SWERVE_DRIVE_MOTOR_KI;
        driveFalconConfiguration.slot0.kD = SWERVE_DRIVE_MOTOR_KD;
        driveFalconConfiguration.slot0.kF = SWERVE_DRIVE_MOTOR_KF;
        driveFalconConfiguration.voltageCompSaturation = SWERVE_VOLTAGE_COMPENSATION;
        driveFalconConfiguration.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                SWERVE_DRIVE_CURRENT_LIMIT_ENABLE,
                SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT,
                SWERVE_DRIVE_PEAK_CURRENT_LIMIT,
                SWERVE_DRIVE_PEAK_CURRENT_DURATION);
        driveFalconConfiguration.openloopRamp = SWERVE_DRIVE_MOTOR_OPENLOOPRAMP;
        driveFalconConfiguration.closedloopRamp = SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP;
        driveFalcon.configFactoryDefault();
        driveFalcon.configAllSettings(driveFalconConfiguration);
        driveFalcon.setInverted(swerveTypeConstants.driveMotorInvert);
        driveFalcon.setNeutralMode(DRIVE_NEUTRAL_MODE);
        driveFalcon.setSelectedSensorPosition(0);
    }

    private void configAngleMotor(){
        angleNEO.restoreFactoryDefaults();
        //Set drive motor config.
        angleNEO.getPIDController().setP(swerveTypeConstants.angleKP);
        angleNEO.getPIDController().setI(swerveTypeConstants.angleKI);
        angleNEO.getPIDController().setD(swerveTypeConstants.angleKD);
        angleNEO.getPIDController().setFF(swerveTypeConstants.angleKF);
        angleNEO.setSmartCurrentLimit(SWERVE_ANGLE_CURRENT_LIMIT);
        angleNEO.enableVoltageCompensation(SWERVE_VOLTAGE_COMPENSATION);
        angleNEO.setInverted(swerveTypeConstants.angleMotorInvert);
        angleNEO.setIdleMode(ANGLE_NEUTRAL_MODE);
        angleMotorEncoder.setPositionConversionFactor(360 / swerveTypeConstants.angleGearRatio);
        resetToAbsolute();
    }

    private void configAngleCanCoder(){
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = swerveTypeConstants.canCoderInvert;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        angleCANCoder.configFactoryDefault();
        angleCANCoder.configAllSettings(canCoderConfiguration);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.falconToMPS(driveFalcon.getSelectedSensorVelocity(), SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveFalcon.getSelectedSensorPosition(), SWERVE_WHEEL_CIRCUMFERENCE, swerveTypeConstants.driveGearRatio),
                getAngle()
        );
    }
}
