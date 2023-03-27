package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final IntSupplier povSlowMoveSup;
    private final BooleanSupplier visionAimSup;
    private final PIDController visionAimPID =
            new PIDController(VISION_AIM_KP, VISION_AIM_KI, VISION_AIM_KD);
    public SwerveDriveCommand(
            SwerveSubsystem swerveSubsystem, DoubleSupplier translationSup,
            DoubleSupplier strafeSup, DoubleSupplier rotationSup,
            IntSupplier povSlowMoveSup, BooleanSupplier robotCentricSup,
            BooleanSupplier visionAimSup) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.povSlowMoveSup = povSlowMoveSup;
        this.visionAimSup = visionAimSup;

        visionAimPID.setTolerance(VISION_AIM_TOLERANCE);
        visionAimPID.setIntegratorRange(-VISION_AIM_INTEGRATOR_RANGE, VISION_AIM_INTEGRATOR_RANGE);
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal;
        double strafeVal;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SWERVE_DRIVE_JOYSTICK_DEADBAND);
        switch (povSlowMoveSup.getAsInt()){
            case 0:
                translationVal = SWERVE_POV_MOVE_SPEED;
                strafeVal = 0.0;
                break;
            case 45:
                translationVal = SWERVE_POV_MOVE_SPEED;
                strafeVal = -SWERVE_POV_MOVE_SPEED;
                break;
            case 90:
                translationVal = 0.0;
                strafeVal = -SWERVE_POV_MOVE_SPEED;
                break;
            case 135:
                translationVal = -SWERVE_POV_MOVE_SPEED;
                strafeVal = -SWERVE_POV_MOVE_SPEED;
                break;
            case 180:
                translationVal = -SWERVE_POV_MOVE_SPEED;
                strafeVal = 0.0;
                break;
            case 225:
                translationVal = -SWERVE_POV_MOVE_SPEED;
                strafeVal = SWERVE_POV_MOVE_SPEED;
                break;
            case 270:
                translationVal = 0.0;
                strafeVal = SWERVE_POV_MOVE_SPEED;
                break;
            case 315:
                translationVal = SWERVE_POV_MOVE_SPEED;
                strafeVal = SWERVE_POV_MOVE_SPEED;
                break;
            default:
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SWERVE_DRIVE_JOYSTICK_DEADBAND);
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SWERVE_DRIVE_JOYSTICK_DEADBAND);
        }
        double visionAimNum = LimelightHelpers.getTX(LIMELIGHT_CENTER_NAME);
        if (visionAimSup.getAsBoolean()&&visionAimNum!=0.0) {
            double vision_rotation = visionAimPID.calculate(visionAimNum, 0);
            /* Vision Drive */
            swerveSubsystem.drive(
                    new Translation2d(translationVal, strafeVal).times(SWERVE_MAX_SPEED),
                    vision_rotation * SWERVE_MAX_ANGULAR_VELOCITY,
                    !robotCentricSup.getAsBoolean(),
                    true
            );
        }else {
            /* Drive */
            swerveSubsystem.drive(
                    new Translation2d(translationVal, strafeVal).times(SWERVE_MAX_SPEED),
                    rotationVal * SWERVE_MAX_ANGULAR_VELOCITY,
                    !robotCentricSup.getAsBoolean(),
                    true
            );}
        }
}