// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class LadderSubsystem extends SubsystemBase
{
    private final CANSparkMax ladderMotor_LL =
            new CANSparkMax(LADDER_MOTOR_LOWER_L, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladderMotor_LR =
            new CANSparkMax(LADDER_MOTOR_LOWER_R, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax ladderMotor_U =
            new CANSparkMax(LADDER_MOTOR_UPPER, CANSparkMaxLowLevel.MotorType.kBrushless);
    public LadderSubsystem(){
        configLowerLadderMotor();
        configUpperLadderMotor();
    }
    private boolean ladderZeroing = false;
    private void setLadderZeroing(){
        if(!ladderZeroing&&!ladderMotor_LL.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed()){
            ladderZeroing = true;
        }
    }
    private void setLadderMotor(double length){
        ladderMotor_LL.getPIDController()
                .setReference(length, CANSparkMax.ControlType.kSmartMotion,0);
    }
    @Override
    public void periodic()
    {
        setLadderZeroing();
    }

    private void configLowerLadderMotor(){
        ladderMotor_LL.restoreFactoryDefaults();
        ladderMotor_LL.setInverted(LADDER_LOWER_MOTOR_INVERTED);
        ladderMotor_LL.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)
                .enableLimitSwitch(false);
        ladderMotor_LL.setSmartCurrentLimit(LADDER_LOWER_MOTOR_CURRENTLIMIT);
        ladderMotor_LL.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_LL.setIdleMode(LADDER_LOWER_MOTOR_IDLEMODE);
        ladderMotor_LL.getEncoder().setPositionConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LL.getPIDController().setP(LADDER_LOWER_MOTOR_KP, 0);
        ladderMotor_LL.getPIDController().setI(LADDER_LOWER_MOTOR_KI, 0);
        ladderMotor_LL.getPIDController().setD(LADDER_LOWER_MOTOR_KD, 0);
        ladderMotor_LL.getPIDController().setFF(LADDER_LOWER_MOTOR_KF,0);
        ladderMotor_LL.getPIDController()
                .setSmartMotionMaxVelocity(LADDER_LOWER_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        ladderMotor_LL.getPIDController()
                .setSmartMotionMaxAccel(LADDER_LOWER_MOTOR_SMARTMOTION_MAX_ACCEL,0);

        ladderMotor_LR.restoreFactoryDefaults();
        ladderMotor_LR.setInverted(LADDER_LOWER_MOTOR_INVERTED);
        ladderMotor_LR.setSmartCurrentLimit(LADDER_LOWER_MOTOR_CURRENTLIMIT);
        ladderMotor_LR.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_LR.setIdleMode(LADDER_LOWER_MOTOR_IDLEMODE);
        ladderMotor_LR.getEncoder().setPositionConversionFactor(LADDER_LOWER_MOTOR_FACTOR);
        ladderMotor_LR.getPIDController().setP(LADDER_LOWER_MOTOR_KP, 0);
        ladderMotor_LR.getPIDController().setI(LADDER_LOWER_MOTOR_KI, 0);
        ladderMotor_LR.getPIDController().setD(LADDER_LOWER_MOTOR_KD, 0);
        ladderMotor_LR.getPIDController().setFF(LADDER_LOWER_MOTOR_KF,0);
        ladderMotor_LR.getPIDController()
                .setSmartMotionMaxVelocity(LADDER_LOWER_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        ladderMotor_LR.getPIDController()
                .setSmartMotionMaxAccel(LADDER_LOWER_MOTOR_SMARTMOTION_MAX_ACCEL,0);

        ladderMotor_LR.follow(ladderMotor_LL, true);
        ladderMotor_LL.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 5);

        ladderMotor_LL.burnFlash();
        ladderMotor_LR.burnFlash();
    }
    private void configUpperLadderMotor(){
        ladderMotor_U.restoreFactoryDefaults();
        ladderMotor_U.setInverted(LADDER_UPPER_MOTOR_INVERTED);
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
    }
}
