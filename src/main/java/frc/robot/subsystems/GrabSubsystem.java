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

public class GrabSubsystem extends SubsystemBase
{
    private final CANSparkMax grab_Angle =
            new CANSparkMax(GRAB_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid intake_Grab = new Solenoid(PNEUMATICS_MODULE_TYPE,GRAB_SOLENOID);

    public GrabSubsystem(){
        configAngleMotor();
        Timer.delay(0.5);
        grab_Angle.getEncoder().setPosition(
                grab_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition());
    }
    private void configAngleMotor(){
        grab_Angle.restoreFactoryDefaults();
        grab_Angle.setInverted(GRAB_ANGLE_MOTOR_INVERTED);
        grab_Angle.setSmartCurrentLimit(GRAB_ANGLE_MOTOR_CURRENTLIMIT);
        grab_Angle.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        grab_Angle.setIdleMode(GRAB_ANGLE_MOTOR_IDLEMODE);
        grab_Angle.getEncoder().setPositionConversionFactor(GRAB_ANGLE_MOTOR_FACTOR);
        grab_Angle.getPIDController().setP(GRAB_ANGLE_MOTOR_KP, 0);
        grab_Angle.getPIDController().setI(GRAB_ANGLE_MOTOR_KI, 0);
        grab_Angle.getPIDController().setD(GRAB_ANGLE_MOTOR_KD, 0);
        grab_Angle.getPIDController().setFF(GRAB_ANGLE_MOTOR_KF,0);
        grab_Angle.getPIDController()
                .setSmartMotionMaxVelocity(GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        grab_Angle.getPIDController()
                .setSmartMotionMaxAccel(GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_ACCEL,0);
        grab_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setInverted(GRAB_ABSOLUTE_ENCODER_INVERTED);
        grab_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setPositionConversionFactor(GRAB_ABSOLUTE_ENCODER_FACTOR);
        grab_Angle.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setZeroOffset(GRAB_ABSOLUTE_ENCODER_OFFSET);

        grab_Angle.burnFlash();
    }

    private void setIntakeAngle(double degree){
        grab_Angle.getPIDController().setReference(degree, CANSparkMax.ControlType.kSmartMotion, 1);
    }
    private void setIntake_Grab(boolean grab){
        intake_Grab.set(grab);
    }
    @Override
    public void periodic()
    {
    }
}
