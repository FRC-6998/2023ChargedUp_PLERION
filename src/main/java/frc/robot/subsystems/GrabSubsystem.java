// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class GrabSubsystem extends SubsystemBase
{
    public final CANSparkMax grab_AngleMotor =
            new CANSparkMax(GRAB_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final Solenoid Grab = new Solenoid(PNEUMATICS_MODULE_TYPE, GRAB_SOLENOID);
    PneumaticHub PH = new PneumaticHub(1);
    private ArmFeedforward grab_ArmFeedforward = new ArmFeedforward(GRAB_ARMFEEDFORWARD_KS,
            GRAB_ARMFEEDFORWARD_KG, GRAB_ARMFEEDFORWARD_KV, GRAB_ARMFEEDFORWARD_KA);
    public boolean grab = false;
    private boolean pressureFulling = false;

    public GrabSubsystem(){
        configAngleMotor();
        resetAngleMotorEncoder();
        PH.enableCompressorAnalog(67.5, 120);
    }
    private void resetAngleMotorEncoder(){
        Timer.delay(0.5);
        double absoluteAngle = grab_AngleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        if(absoluteAngle<250){
            grab_AngleMotor.getEncoder().setPosition(absoluteAngle);
        }else{
            grab_AngleMotor.getEncoder().setPosition(absoluteAngle-360);
        }
    }
    private void configAngleMotor(){
        grab_AngleMotor.restoreFactoryDefaults();
        grab_AngleMotor.setInverted(GRAB_ANGLE_MOTOR_INVERTED);
        grab_AngleMotor
                .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
        grab_AngleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, GRAB_ANGLE_LIMIT);
        grab_AngleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        grab_AngleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        grab_AngleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        grab_AngleMotor.setSmartCurrentLimit(GRAB_ANGLE_MOTOR_CURRENTLIMIT);
        grab_AngleMotor.enableVoltageCompensation(UP_VOLTAGE_COMPENSATION);
        grab_AngleMotor.setIdleMode(GRAB_ANGLE_MOTOR_IDLEMODE);
        grab_AngleMotor.getEncoder().setPositionConversionFactor(GRAB_ANGLE_MOTOR_FACTOR);
        grab_AngleMotor.getEncoder().setVelocityConversionFactor(GRAB_ANGLE_MOTOR_FACTOR);
        grab_AngleMotor.getPIDController().setP(GRAB_ANGLE_MOTOR_KP, 0);
        grab_AngleMotor.getPIDController().setI(GRAB_ANGLE_MOTOR_KI, 0);
        grab_AngleMotor.getPIDController().setD(GRAB_ANGLE_MOTOR_KD, 0);
        grab_AngleMotor.getPIDController()
                .setSmartMotionMaxVelocity(GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_VELOCITY,0);
        grab_AngleMotor.getPIDController()
                .setSmartMotionMaxAccel(GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_ACCEL,0);
        grab_AngleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setInverted(GRAB_ABSOLUTE_ENCODER_INVERTED);
        grab_AngleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setPositionConversionFactor(GRAB_ABSOLUTE_ENCODER_FACTOR);
        grab_AngleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setZeroOffset(GRAB_ABSOLUTE_ENCODER_OFFSET);
        grab_AngleMotor.burnFlash();
    }

    public void setGrabAngle(double degree){
        grab_AngleMotor.getPIDController().setReference(
                degree, CANSparkMax.ControlType.kPosition, 0, grab_ArmFeedforward.calculate(degree, 0));
    }
    public void set_Grabing(){
        grab = !grab;
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("workingPressure", PH.getPressure(1));
        SmartDashboard.putNumber("StorePressure", PH.getPressure(0));
        Grab.set(grab);
    }
}
