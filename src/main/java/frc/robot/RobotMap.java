package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;

public class RobotMap {
    public static final String SWERVE_CANBUS_TYPE = "rio";
    public static final PneumaticsModuleType  PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

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

    public static final int INTAKE_ANGLE_MOTOR = 5;

    public static final int INTAKE_LADDER_MOTOR_L = 6;
    public static final int INTAKE_LADDER_MOTOR_R = 7;
}
