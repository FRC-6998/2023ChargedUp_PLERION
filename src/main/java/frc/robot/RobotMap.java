package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {
    public static final String SWERVE_CANBUS_TYPE = "rio";
    public static final PneumaticsModuleType  PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
    public static final SerialPort.Port NAVX_SERIAL_TYPE = SerialPort.Port.kUSB;

    public static final String LIMELIGHT_SIDE_NAME = "limelight-side";
    public static final String LIMELIGHT_FRONT_NAME = "limelight-front";

    public static final int SWERVE_LEFTFRONT_DRIVEMOTOR = 1;
    public static final int SWERVE_LEFTFRONT_ANGLEMOTOR = 1;
    public static final int SWERVE_LEFTFRONT_CANCODER = 1;

    public static final int SWERVE_LEFTREAR_DRIVEMOTOR = 2;
    public static final int SWERVE_LEFTREAR_ANGLEMOTOR = 2;
    public static final int SWERVE_LEFTREAR_CANCODER = 2;

    public static final int SWERVE_RIGHTFRONT_DRIVEMOTOR = 4;
    public static final int SWERVE_RIGHTFRONT_ANGLEMOTOR = 4;
    public static final int SWERVE_RIGHTFRONT_CANCODER = 4;

    public static final int SWERVE_RIGHTREAR_DRIVEMOTOR = 3;
    public static final int SWERVE_RIGHTREAR_ANGLEMOTOR = 3;
    public static final int SWERVE_RIGHTREAR_CANCODER = 3;

    public static final int GRAB_ANGLE_MOTOR = 8;
    public static final int GRAB_SOLENOID = 0;
    public static final int LADDER_MOTOR_LOWER_L = 5;
    public static final int LADDER_MOTOR_LOWER_R = 6;
    public static final int LADDER_MOTOR_UPPER = 7;
}
