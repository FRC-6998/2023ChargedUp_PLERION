// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final Rotation2d SWERVE_LEFTFRONT_OFFSET = Rotation2d.fromDegrees(169.365234375+180);//wait for test
    public static final Rotation2d SWERVE_LEFTREAR_OFFSET = Rotation2d.fromDegrees(241.61132812500003-180);
    public static final Rotation2d SWERVE_RIGHTFRONT_OFFSET = Rotation2d.fromDegrees(253.38867187499997-180);
    public static final Rotation2d SWERVE_RIGHTREAR_OFFSET = Rotation2d.fromDegrees(45.70312500000001);

    public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.62865;
    public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.62865;
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MAX_VOTAGE = 12.0;
    public static final double SWERVE_MAX_SPEED = 4.5;//Wait for test.
    public static final double SWERVE_MAX_ANGULAR_VELOCITY = 4.5;//Wait for test.

    public static final double SWERVE_POV_MOVE_SPEED = 0.2;


    public static final double SWERVE_VOLTAGE_COMPENSATION = 12.0;
    //Swerve angle falcon current limit.
    public static final int SWERVE_ANGLE_CURRENT_LIMIT = 35;

    public static final CANSparkMax.IdleMode ANGLE_NEUTRAL_MODE = CANSparkMax.IdleMode.kCoast;

    //Swerve drive falcon current limit.
    public static final int SWERVE_DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int SWERVE_DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double SWERVE_DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean SWERVE_DRIVE_CURRENT_LIMIT_ENABLE = true;

    public static final double SWERVE_DRIVE_MOTOR_KP = 0.05;//Wait to test.
    public static final double SWERVE_DRIVE_MOTOR_KI = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_KD = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_KF = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_OPENLOOPRAMP = 0.25;
    public static final double SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP = 0.0;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    public static final double SWERVE_DRIVE_KS = (0.068817 / MAX_VOTAGE);//Wait to test by SYSID.
    public static final double SWERVE_DRIVE_KV = (2.755 / MAX_VOTAGE);
    public static final double SWERVE_DRIVE_KA = (0.73295 / MAX_VOTAGE);

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */

    public static final boolean NAVX_INVERTED = true;

    public static final double SWERVE_DRIVE_JOYSTICK_DEADBAND = 0.05;

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, -SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, -SWERVE_CHASSIS_WHEELBASE_METERS / 2.0)
    );

    public static final double SWERVE_AUTO_XY_KP = 5.0;
    public static final double SWERVE_AUTO_XY_KI = 0.0;
    public static final double SWERVE_AUTO_XY_KD = 0.0;

    public static final double SWERVE_AUTO_Z_KP = 0.01;
    public static final double SWERVE_AUTO_Z_KI = 0.0;
    public static final double SWERVE_AUTO_Z_KD = 0.0;

    public static final double AUTO_BALANCE_KP = 0.0045;
    public static final double AUTO_BALANCE_KI = 0;
    public static final double AUTO_BALANCE_KD = 0;
    public static final double AUTO_BALANCE_TOLERANCE = 1.5;
    public static final double AUTO_PREPARE_CHANGENUM = 7;
    public static final double AUTO_BALANCE_WAIT_TIME = 2;
    public static final double AUTO_BALANCE_PREPARING_SPEED = 0.35;

    public static final double UP_VOLTAGE_COMPENSATION = 12.0;


    public static final boolean GRAB_ANGLE_MOTOR_INVERTED = false;
    public static final int GRAB_ANGLE_MOTOR_CURRENTLIMIT = 35;
    public static final double GRAB_ANGLE_MOTOR_FACTOR = 360.0 / 80.0;
    public static final CANSparkMax.IdleMode GRAB_ANGLE_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double GRAB_ANGLE_MOTOR_KP = 1;//Wait to test.
    public static final double GRAB_ANGLE_MOTOR_KI = 0.0;
    public static final double GRAB_ANGLE_MOTOR_KD = 0.019669;
    public static final double GRAB_ARMFEEDFORWARD_KS = 0.10479;
    public static final double GRAB_ARMFEEDFORWARD_KV = 0.026641;
    public static final double GRAB_ARMFEEDFORWARD_KA = 0.0004418;
    public static final double GRAB_ARMFEEDFORWARD_KG = 0.19158;
    public static final double GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_VELOCITY = 5870 * GRAB_ANGLE_MOTOR_FACTOR;
    public static final double GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_ACCEL = 5870 * GRAB_ANGLE_MOTOR_FACTOR;
    public static final boolean GRAB_ABSOLUTE_ENCODER_INVERTED = true;
    public static final double GRAB_ABSOLUTE_ENCODER_FACTOR = 360.0;
    public static final double GRAB_ABSOLUTE_ENCODER_OFFSET = 290;
    public static final float GRAB_ANGLE_LIMIT = 110;


    public static final boolean LADDER_LOWER_MOTOR_INVERTED = false;
    public static final int LADDER_LOWER_MOTOR_CURRENTLIMIT = 35;
    public static final double LADDER_LOWER_MOTOR_FACTOR = 1.0/30.0;
    public static final CANSparkMax.IdleMode LADDER_LOWER_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double LADDER_LOWER_MOTOR_KP = 0.952233333333333;
    public static final double LADDER_LOWER_MOTOR_KI = 0.0;
    public static final double LADDER_LOWER_MOTOR_KD = 0.20268;
    public static final double LADDER_LOWER_MOTOR_KF = 0.00535714286;
    public static final float LADDER_LOWER_MAX_LENGTH = 3.500f;

    public static final boolean LADDER_UPPER_MOTOR_INVERTED = false;
    public static final int LADDER_UPPER_MOTOR_CURRENTLIMIT = 35;
    public static final double LADDER_UPPER_MOTOR_FACTOR = 1/35.0;
    public static final CANSparkMax.IdleMode LADDER_UPPER_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double LADDER_UPPER_MOTOR_KP = 1.31380952;
    public static final double LADDER_UPPER_MOTOR_KI = 0.0;
    public static final double LADDER_UPPER_MOTOR_KD = 0.0526047619;
    public static final double LADDER_UPPER_MOTOR_KF = 0.0001875;
    public static final double LADDER_UPPER_MOTOR_SMARTMOTION_MAX_VELOCITY = 190;
    public static final double LADDER_UPPER_MOTOR_SMARTMOTION_MAX_ACCEL = 1000;
    public static final double LADDER_UPPER_MOTOR_SMARTMOTION_ALLOWEDERRORS = 0;
    public static final float LADDER_UPPER_MAX_LENGTH = 2.45f;


    public static final double LADDER_MAX_LENGTH = LADDER_LOWER_MAX_LENGTH + LADDER_UPPER_MAX_LENGTH;


    public static final double LADDER_POVCONTROL_WAITTIME = 1;
    public static final double LADDER_POVCONTROL_NUM = 0.2;
    public static final double START_GRAB_ANGLE = 20;
    public static final double GRAB_ANGLECONTROL_WAITTIME = 0.005;
    public static final double GRAB_ANGLECONTROL_NUM = 1.375;

    public static final double VISION_POSE_TRUST_WORTHINESS = 1;

    public static final double VISION_AIM_KP = 0.01;
    public static final double VISION_AIM_KI = 0;
    public static final double VISION_AIM_KD = 0;
    public static  final  double VISION_AIM_TOLERANCE = 0;
    public static final double VISION_AIM_INTEGRATOR_RANGE = 0.5;
}
