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
    private final CANSparkMax ladderMotor_L =
            new CANSparkMax(LADDER_MOTOR_L, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax ladderMotor_R =
            new CANSparkMax(LADDER_MOTOR_R, CANSparkMaxLowLevel.MotorType.kBrushless);
    public LadderSubsystem(){
        configLadderMotor();
    }
    private boolean ladderZeroing = false;
    private void configLadderMotor(){
        ladderMotor_L.restoreFactoryDefaults();
        ladderMotor_L.setInverted(LADDER_MOTOR_INVERTED);
        ladderMotor_L.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed)
                .enableLimitSwitch(true);
        ladderMotor_L.setSmartCurrentLimit(LADDER_MOTOR_CURRENTLIMIT);
        ladderMotor_L.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        ladderMotor_L.setIdleMode(LADDER_MOTOR_IDLEMODE);
        ladderMotor_L.getEncoder().setPositionConversionFactor(LADDER_MOTOR_FACTOR);
        ladderMotor_L.getPIDController().setP(LADDER_MOTOR_KP, 0);
        ladderMotor_L.getPIDController().setI(LADDER_MOTOR_KI, 0);
        ladderMotor_L.getPIDController().setD(LADDER_MOTOR_KD, 0);
        ladderMotor_L.getPIDController().setFF(LADDER_MOTOR_KF,0);
        ladderMotor_L.getPIDController()
                .setSmartMotionMaxVelocity(LADDER_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        ladderMotor_L.getPIDController()
                .setSmartMotionMaxAccel(LADDER_MOTOR_SMARTMOTION_MAX_ACCEL,0);
    }
    private void setLadderZeroing(){
        if(!ladderZeroing&&!ladderMotor_L.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed).isPressed()){
            ladderZeroing =true;
        }
    }
    @Override
    public void periodic()
    {
    }
}
