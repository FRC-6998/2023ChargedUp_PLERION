// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class LadderSubsystem extends SubsystemBase
{
    private GrabSubsystem grabSubsystem;
    private final CANSparkMax ladderMotor_LL =
            new CANSparkMax(LADDER_MOTOR_LOWER_L, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladderMotor_LR =
            new CANSparkMax(LADDER_MOTOR_LOWER_R, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladderMotor_U =
            new CANSparkMax(LADDER_MOTOR_UPPER, CANSparkMaxLowLevel.MotorType.kBrushless);
    public LadderSubsystem(GrabSubsystem grabSubsystem){
        this.grabSubsystem = grabSubsystem;
        configLowerLadderMotor();
        configUpperLadderMotor();
    }
    private boolean UpperladderZeroing = false;
    private boolean LowerladderZeroing = false;
    private void setLadderZeroing(){
        if(!LowerladderZeroing&&!ladderMotor_LL.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed()){
            ladderMotor_LL.getEncoder().setPosition(0);
            ladderMotor_LR.getEncoder().setPosition(0);
            LowerladderZeroing = true;
        }
        if(!UpperladderZeroing&&!grabSubsystem.grab_AngleMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed()){
            ladderMotor_U.getEncoder().setPosition(0);
            UpperladderZeroing = true;
        }
    }
    public void setLadderLength(double length){
        ladderMotor_LL.getPIDController().setReference(
                length * LADDER_LOWER_MAX_LENGTH/LADDER_MAX_LENGTH, CANSparkMax.ControlType.kPosition);
        ladderMotor_U.getPIDController().setReference(
                length * LADDER_UPPER_MAX_LENGTH/LADDER_MAX_LENGTH, CANSparkMax.ControlType.kPosition);
    }
    @Override
    public void periodic()
    {
        setLadderZeroing();
        SmartDashboard.putBoolean("L", LowerladderZeroing);
        SmartDashboard.putBoolean("U", UpperladderZeroing);
    }

    private void configLowerLadderMotor(){
        ladderMotor_LL.restoreFactoryDefaults();
        ladderMotor_LL.setInverted(LADDER_LOWER_MOTOR_INVERTED);
        ladderMotor_LL.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)
                .enableLimitSwitch(false);
        ladderMotor_LL.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LADDER_LOWER_MAX_LENGTH);
        ladderMotor_LL.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladderMotor_LL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladderMotor_LL.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladderMotor_LL.setSmartCurrentLimit(LADDER_LOWER_MOTOR_CURRENTLIMIT);
        ladderMotor_LL.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_LL.setIdleMode(LADDER_LOWER_MOTOR_IDLEMODE);
        ladderMotor_LL.getEncoder().setPositionConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LL.getEncoder().setVelocityConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LL.getPIDController().setP(LADDER_LOWER_MOTOR_KP, 0);
        ladderMotor_LL.getPIDController().setI(LADDER_LOWER_MOTOR_KI, 0);
        ladderMotor_LL.getPIDController().setD(LADDER_LOWER_MOTOR_KD, 0);
        ladderMotor_LL.getPIDController().setFF(LADDER_LOWER_MOTOR_KF,0);
        ladderMotor_LL.getPIDController()
                .setSmartMotionAllowedClosedLoopError(LADDER_UPPER_MOTOR_SMARTMOTION_ALLOWEDERRORS, 0);

        ladderMotor_LR.restoreFactoryDefaults();
        ladderMotor_LR.setInverted(LADDER_LOWER_MOTOR_INVERTED);
        ladderMotor_LR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LADDER_LOWER_MAX_LENGTH);
        ladderMotor_LR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladderMotor_LR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladderMotor_LR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladderMotor_LR.setSmartCurrentLimit(LADDER_LOWER_MOTOR_CURRENTLIMIT);
        ladderMotor_LR.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_LR.setIdleMode(LADDER_LOWER_MOTOR_IDLEMODE);
        ladderMotor_LR.getEncoder().setPositionConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LR.getEncoder().setVelocityConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LR.getPIDController().setP(LADDER_LOWER_MOTOR_KP, 0);
        ladderMotor_LR.getPIDController().setI(LADDER_LOWER_MOTOR_KI, 0);
        ladderMotor_LR.getPIDController().setD(LADDER_LOWER_MOTOR_KD, 0);
        ladderMotor_LR.getPIDController().setFF(LADDER_LOWER_MOTOR_KF,0);
        ladderMotor_LR.getPIDController()
                .setSmartMotionAllowedClosedLoopError(LADDER_UPPER_MOTOR_SMARTMOTION_ALLOWEDERRORS, 0);

        ladderMotor_LR.follow(ladderMotor_LL, true);
        ladderMotor_LL.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 5);

        ladderMotor_LL.burnFlash();
        ladderMotor_LR.burnFlash();
    }
    private void configUpperLadderMotor(){
        ladderMotor_U.restoreFactoryDefaults();
        ladderMotor_U.setInverted(LADDER_UPPER_MOTOR_INVERTED);
        ladderMotor_U.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, LADDER_UPPER_MAX_LENGTH);
        ladderMotor_U.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        ladderMotor_U.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        ladderMotor_U.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        ladderMotor_U.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)
                .enableLimitSwitch(false);
        ladderMotor_U.setSmartCurrentLimit(LADDER_UPPER_MOTOR_CURRENTLIMIT);
        ladderMotor_U.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_U.setIdleMode(LADDER_UPPER_MOTOR_IDLEMODE);
        ladderMotor_U.getEncoder().setPositionConversionFactor(LADDER_UPPER_MOTOR_FACTOR);
        ladderMotor_U.getPIDController().setP(LADDER_UPPER_MOTOR_KP, 0);
        ladderMotor_U.getPIDController().setI(LADDER_UPPER_MOTOR_KI, 0);
        ladderMotor_U.getPIDController().setD(LADDER_UPPER_MOTOR_KD, 0);
        ladderMotor_U.getPIDController().setFF(LADDER_UPPER_MOTOR_KF,0);
        ladderMotor_U.getPIDController()
                .setSmartMotionMaxVelocity(LADDER_UPPER_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        ladderMotor_U.getPIDController()
                .setSmartMotionMaxAccel(LADDER_UPPER_MOTOR_SMARTMOTION_MAX_ACCEL,0);
        ladderMotor_U.getPIDController()
                .setSmartMotionAllowedClosedLoopError(LADDER_UPPER_MOTOR_SMARTMOTION_ALLOWEDERRORS, 0);

        ladderMotor_U.burnFlash();
    }
}
