package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveTypeConstants;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveDriveOdometry;
    public SwerveModule[] swerveModules;
    public AHRS navX;
    private boolean isBrakeForCharge = false;
    public SwerveSubsystem(){
        navX = new AHRS(NAVX_SERIAL_TYPE);
        navX.reset();

        swerveModules = new SwerveModule[]{
                new SwerveModule(
                        0, SwerveTypeConstants.SDSMK4I_L1(),
                        SWERVE_LEFTFRONT_DRIVEMOTOR,SWERVE_LEFTFRONT_ANGLEMOTOR,SWERVE_LEFTFRONT_CANCODER,
                        SWERVE_LEFTFRONT_OFFSET),
                new SwerveModule(
                        1, SwerveTypeConstants.SDSMK4I_L1(),
                        SWERVE_LEFTREAR_DRIVEMOTOR,SWERVE_LEFTREAR_ANGLEMOTOR,SWERVE_LEFTREAR_CANCODER,
                        SWERVE_LEFTREAR_OFFSET),
                new SwerveModule(
                        2, SwerveTypeConstants.SDSMK4I_L1(),
                        SWERVE_RIGHTREAR_DRIVEMOTOR,SWERVE_RIGHTREAR_ANGLEMOTOR,SWERVE_RIGHTREAR_CANCODER,
                        SWERVE_RIGHTREAR_OFFSET),
                new SwerveModule(
                        3, SwerveTypeConstants.SDSMK4I_L1(),
                        SWERVE_RIGHTFRONT_DRIVEMOTOR,SWERVE_RIGHTFRONT_ANGLEMOTOR,SWERVE_RIGHTFRONT_CANCODER,
                        SWERVE_RIGHTFRONT_OFFSET)

        };
        Timer.delay(1);
        resetModulesToAbsolute();

        swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                swerveDriveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getYaw()
                        )
                                : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SWERVE_MAX_SPEED);

        if(!isBrakeForCharge){
            for(SwerveModule mod : swerveModules){
                mod.setDesiredState(swerveModuleStates[mod.moduleNum], isOpenLoop);
            }
        }

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWERVE_MAX_SPEED);

        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNum], false);
        }
    }

    public Pose2d getPose() {
        return swerveDriveOdometry.getPoseMeters();
    }
    public Rotation2d getRotation2d() {
        return navX.getRotation2d();
    }
    public void resetOdometry(Pose2d pose) {
        swerveDriveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveModules){
            states[mod.moduleNum] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            positions[mod.moduleNum] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        navX.reset();
    }
    public void setNavXYaw(double yaw){
        navX.setAngleAdjustment(yaw);
    }

    public Rotation2d getYaw() {
        return (NAVX_INVERTED) ? Rotation2d.fromDegrees(360 - navX.getYaw()) : Rotation2d.fromDegrees(navX.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveModules){
            mod.resetToAbsolute();
        }
    }

    public void setBrakingForCharge(){isBrakeForCharge = !isBrakeForCharge;}
    private void brakeForCharge(){
        swerveModules[0].setNoFilterAngle(45);
        swerveModules[1].setNoFilterAngle(135);
        swerveModules[2].setNoFilterAngle(45);
        swerveModules[3].setNoFilterAngle(135);
        for(SwerveModule mod : swerveModules){
            mod.setNoMove();
        }
    }

    @Override
    public void periodic(){
        if(isBrakeForCharge){brakeForCharge();}
        swerveDriveOdometry.update(getYaw(), getModulePositions());
        for(SwerveModule mod : swerveModules){
            SmartDashboard.putNumber("Mod " + mod.moduleNum + " CanCoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNum + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNum + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}
