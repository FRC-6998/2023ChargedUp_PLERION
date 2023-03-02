// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final Rotation2d SWERVE_LEFTFRONT_OFFSET = Rotation2d.fromDegrees(0);//wait for test
    public static final Rotation2d SWERVE_LEFTREAR_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d SWERVE_RIGHTFRONT_OFFSET = Rotation2d.fromDegrees(0);
    public static final Rotation2d SWERVE_RIGHTREAR_OFFSET = Rotation2d.fromDegrees(0);

    public static final double SWERVE_CHASSIS_TRACKWIDTH_METERS = 0.62865;
    public static final double SWERVE_CHASSIS_WHEELBASE_METERS = 0.62865;
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MAX_VOTAGE = 12.0;
    public static final double SWERVE_MAX_SPEED = 4.5;//Wait for test.
    public static final double SWERVE_MAX_ANGULAR_VELOCITY = 4.5;//Wait for test.

    public static final double SWERVE_POV_MOVE_SPEED = 0.15;


    public static final double SWERVE_VOLTAGE_COMPENSATION = 12.0;
    //Swerve angle falcon current limit.
    public static final int SWERVE_ANGLE_CURRENT_LIMIT = 35;
    public static final boolean SWERVE_ANGLE_CURRENT_LIMIT_ENABLE = true;

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

    public static final double SWERVE_DRIVE_KS = (0.32 / MAX_VOTAGE);//Wait to test by SYSID.
    public static final double SWERVE_DRIVE_KV = (1.51 / MAX_VOTAGE);
    public static final double SWERVE_DRIVE_KA = (0.27 / MAX_VOTAGE);

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */

    public static final boolean NAVX_INVERTED = true;

    public static final double SWERVE_DRIVE_JOYSTICK_DEADBAND = 0.05;

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, -SWERVE_CHASSIS_WHEELBASE_METERS / 2.0),
            new Translation2d(-SWERVE_CHASSIS_TRACKWIDTH_METERS / 2.0, -SWERVE_CHASSIS_WHEELBASE_METERS / 2.0)
    );

    public static final double SWERVE_AUTO_XY_KP = 5.0;
    public static final double SWERVE_AUTO_XY_KI = 0.0;
    public static final double SWERVE_AUTO_XY_KD = 0.0;

    public static final double SWERVE_AUTO_Z_KP = 0.0;
    public static final double SWERVE_AUTO_Z_KI = 0.0;
    public static final double SWERVE_AUTO_Z_KD = 0.0;

    public static final double AUTO_BALANCE_KP = 0.2;
    public static final double AUTO_BALANCE_KI = 0;
    public static final double AUTO_BALANCE_KD = 0;
    public static final double AUTO_BALANCE_TOLERANCE = 3;
    public static final double AUTO_BALANCE_WAIT_TIME = 2;

    public static final double UP_VOLTAGE_COMPENSATION = 12.0;


    public static final boolean GRAB_ANGLE_MOTOR_INVERTED = false;
    public static final int GRAB_ANGLE_MOTOR_CURRENTLIMIT = 35;
    public static final double GRAB_ANGLE_MOTOR_FACTOR = 360.0 / 80.0;
    public static final CANSparkMax.IdleMode GRAB_ANGLE_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double GRAB_ANGLE_MOTOR_KP = 0.05;//Wait to test.
    public static final double GRAB_ANGLE_MOTOR_KI = 0.0;
    public static final double GRAB_ANGLE_MOTOR_KD = 0.0;
    public static final double GRAB_ANGLE_MOTOR_KF = 0.0;
    public static final double GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_VELOCITY = 5870 * GRAB_ANGLE_MOTOR_FACTOR;
    public static final double GRAB_ANGLE_MOTOR_SMARTMOTION_MAX_ACCEL = 5870 * GRAB_ANGLE_MOTOR_FACTOR;
    public static final boolean GRAB_ABSOLUTE_ENCODER_INVERTED = false;
    public static final double GRAB_ABSOLUTE_ENCODER_FACTOR = 360.0;
    public static final double GRAB_ABSOLUTE_ENCODER_OFFSET = 360.0;


    public static final boolean LADDER_MOTOR_INVERTED = false;
    public static final int LADDER_MOTOR_CURRENTLIMIT = 35;
    public static final double LADDER_MOTOR_FACTOR = 30;
    public static final CANSparkMax.IdleMode LADDER_MOTOR_IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double LADDER_MOTOR_KP = 0.05;//Wait to test.
    public static final double LADDER_MOTOR_KI = 0.0;
    public static final double LADDER_MOTOR_KD = 0.0;
    public static final double LADDER_MOTOR_KF = 0.0;
    public static final double LADDER_MOTOR_SMARTMOTION_MAX_VELOCITY = 5870 * LADDER_MOTOR_FACTOR;
    public static final double LADDER_MOTOR_SMARTMOTION_MAX_ACCEL = 20000;
    public static final double LADDER_MAX_LENGTH = 1;

    public static final double LADDER_POVCONTROL_PLUSTIME = 1;
}
