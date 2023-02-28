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

public class IntakeSubsystem extends SubsystemBase
{
    private final CANSparkMax intake_Angle =
            new CANSparkMax(INTAKE_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final Solenoid intake_Grab = new Solenoid(PNEUMATICS_MODULE_TYPE,0);

    public IntakeSubsystem(){
        configAngleMotor();
        Timer.delay(0.5);
        intake_Angle.getEncoder().setPosition(
                intake_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }
    private void configAngleMotor(){
        intake_Angle.restoreFactoryDefaults();
        intake_Angle.setInverted(INTAKE_ANGLE_MOTOR_INVERTED);
        intake_Angle.setSmartCurrentLimit(INTAKE_ANGLE_MOTOR_CURRENTLIMIT);
        intake_Angle.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        intake_Angle.setIdleMode(INTAKE_ANGLE_MOTOR_IDLEMODE);
        intake_Angle.getEncoder().setPositionConversionFactor(INTAKE_ANGLE_MOTOR_FACTOR);
        intake_Angle.getPIDController().setP(INTAKE_ANGLE_MOTOR_KP, 0);
        intake_Angle.getPIDController().setI(INTAKE_ANGLE_MOTOR_KI, 0);
        intake_Angle.getPIDController().setD(INTAKE_ANGLE_MOTOR_KD, 0);
        intake_Angle.getPIDController().setFF(INTAKE_ANGLE_MOTOR_KF,0);
        intake_Angle.getPIDController()
                .setSmartMotionMaxVelocity(INTAKE_ANGLE_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        intake_Angle.getPIDController()
                .setSmartMotionMaxAccel(INTAKE_ANGLE_MOTOR_SMARTMOTION_MAX_ACCEL,0);
        intake_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setInverted(INTAKE_ABSOLUTE_ENCODER_INVERTED);
        intake_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setPositionConversionFactor(INTAKE_ABSOLUTE_ENCODER_FACTOR);
        intake_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setZeroOffset(INTAKE_ABSOLUTE_ENCODER_OFFSET);
    }

    private void setIntakeAngle(double degree){
        intake_Angle.getPIDController().setReference(degree, CANSparkMax.ControlType.kSmartMotion);
    }
    private void setIntake_Grab(boolean grab){
        intake_Grab.set(grab);
    }
    @Override
    public void periodic()
    {
    }
}
