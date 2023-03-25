package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.IOException;

import static frc.robot.Constants.*;
import static frc.robot.RobotMap.*;

public class SwerveEstimatorsystem extends SubsystemBase {
    public Field2d field = new Field2d();
    //Trustworthiness of the internal model of how motors should be moving Measured in expected standard deviation
    //(meters of position and degrees of rotation)
    public Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    //Trustworthiness of the vision system Measured in expected standard deviation
    // (meters of position and degrees of rotation)
    public Matrix<N3, N1> visionMeasurementStdDevs_limelightSide = VecBuilder.fill(2, 2, 2);
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    SwerveSubsystem swerveSubsystem;
    public SwerveEstimatorsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        swerveDrivePoseEstimator =  new SwerveDrivePoseEstimator(
                swerveDriveKinematics,
                swerveSubsystem.getYaw(),
                swerveSubsystem.getModulePositions(),
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                stateStdDevs,
                visionMeasurementStdDevs_limelightSide
        );
        SmartDashboard.putData("Field", field);
    }
    public void resetSwerveDrivePoseEstimator(Pose2d initPose){
        swerveDrivePoseEstimator.resetPosition(swerveSubsystem.getYaw(),swerveSubsystem.getModulePositions(),initPose);
    }
    private void updateOdometry()
    {
        // Update odometry
        swerveDrivePoseEstimator.update(swerveSubsystem.getYaw(), swerveSubsystem.getModulePositions());
        field.setRobotPose(new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(), swerveSubsystem.getYaw()));
    }
    public void addVisionMeasurement()
    {
        try {
            Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_SIDE_NAME);
            double latency = Timer.getFPGATimestamp() - (LimelightHelpers.getBotPose(LIMELIGHT_SIDE_NAME)[6]/1000.0);
                swerveDrivePoseEstimator.addVisionMeasurement(visionPose, latency,
                        visionMeasurementStdDevs_limelightSide.times(1.0 / VISION_POSE_TRUST_WORTHINESS));
        } catch (Exception e) {}

    }
    @Override
    public void periodic(){
        addVisionMeasurement();
        updateOdometry();
    }
}