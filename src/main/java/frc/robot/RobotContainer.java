// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Autos.AutoBalanceCommand;
import frc.robot.commands.Autos.AutoPutCommand;
import frc.robot.subsystems.*;

import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import static frc.robot.Constants.*;


public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LadderSubsystem ladderSubsystem = new LadderSubsystem();
    private final GrabSubsystem grabSubsystem = new GrabSubsystem();
    private final SwerveEstimatorsystem swerveEstimatorsystem = new SwerveEstimatorsystem(swerveSubsystem);

    private final static XboxController controller_driveX = new XboxController(0);
    private final static XboxController controller_Operator = new XboxController(1);
    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(
                swerveSubsystem,
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kLeftX.value),
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kRightX.value),
                controller_driveX::getPOV,
                controller_driveX::getBackButton
        ));

        ladderSubsystem.setDefaultCommand(new LadderControlCommand(
                ladderSubsystem,
                controller_Operator::getPOV,
                controller_Operator::getAButton
        ));

        grabSubsystem.setDefaultCommand(new GrabAngleControlCommand(
                grabSubsystem,
                controller_Operator::getYButton,
                controller_Operator::getBButton
        ));
        // Configure the trigger bindings
        configureBindings();
        if(DriverStation.getAlliance()==DriverStation.Alliance.Blue){
            pathChooser.setDefaultOption("ONLY LONG OUT", "BLUE_LONG_ONLY_OUT");
            pathChooser.setDefaultOption("ONLY SHORT OUT", "BLUE_SHORT_ONLY_OUT");
            pathChooser.addOption("ONLY BALANCE", "BLUE_ONLY_BALANCE");
            pathChooser.addOption("LONG OUT AND BALANCE", "BLUE_LONG_OUT_AND_BALANCE");
            pathChooser.addOption("SHORT OUT AND BALANCE", "BLUE_SHORT_OUT_AND_BALANCE");
            pathChooser.addOption("DONT MOVE", "BLUE_DONT_MOVE");
            SmartDashboard.putData("Auto choices", pathChooser);
        } else if (DriverStation.getAlliance()==DriverStation.Alliance.Red){
            pathChooser.setDefaultOption("ONLY LONG OUT", "RED_LONG_ONLY_OUT");
            pathChooser.setDefaultOption("ONLY SHORT OUT", "RED_SHORT_ONLY_OUT");
            pathChooser.setDefaultOption("ONLY OUT", "RED_ONLY_OUT");
            pathChooser.addOption("ONLY BALANCE", "RED_ONLY_BALANCE");
            pathChooser.addOption("LONG OUT AND BALANCE", "RED_LONG_OUT_AND_BALANCE");
            pathChooser.addOption("SHORT OUT AND BALANCE", "RED_SHORT_OUT_AND_BALANCE");
            pathChooser.addOption("DONT MOVE", "RED_DONT_MOVE");
            SmartDashboard.putData("Auto choices", pathChooser);
        }
    }


    private void configureBindings()
    {
        new JoystickButton(controller_driveX, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        new JoystickButton(controller_Operator, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(grabSubsystem::set_Grabing));
    }

    public void robotInit() {
    }

    public Command getAutonomousCommand() {
        if(pathChooser.getSelected()=="DONT_MOVE"){
            return null;
        } else if (pathChooser.getSelected()=="BLUE_ONLY_BALANCE"||pathChooser.getSelected()=="RED_ONLY_BALANCE") {
            return new AutoBalanceCommand(swerveSubsystem,true, false);
        } else  {
            List<PathPlannerTrajectory> pathGroup =
                    PathPlanner.loadPathGroup(pathChooser.getSelected(), new PathConstraints(4, 3));
            // This is just an example event map. It would be better to have a constant, global event map
            // in your code that will be used by all path following commands
            HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("PUT_CONE_FIRST", new AutoPutCommand("CONE",1, ladderSubsystem, grabSubsystem));
            eventMap.put("PUT_CONE_SECOND", new AutoPutCommand("CONE",2, ladderSubsystem, grabSubsystem));
            eventMap.put("PUT_CONE_THIRD", new AutoPutCommand("CONE",3, ladderSubsystem, grabSubsystem));
            eventMap.put("PUT_CUBE_FIRST", new AutoPutCommand("CUBE",1, ladderSubsystem, grabSubsystem));
            eventMap.put("PUT_CUBE_SECOND", new AutoPutCommand("CUBE",2, ladderSubsystem, grabSubsystem));
            eventMap.put("PUT_CUBE_THIRD", new AutoPutCommand("CUBE",3, ladderSubsystem, grabSubsystem));
            swerveEstimatorsystem.resetSwerveDrivePoseEstimator(pathGroup.get(0).getInitialPose());
            // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
            SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                    swerveEstimatorsystem.swerveDrivePoseEstimator::getEstimatedPosition, // Pose2d supplier
                    swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                    swerveDriveKinematics, // SwerveDriveKinematics
                    new PIDConstants(SWERVE_AUTO_XY_KP, SWERVE_AUTO_XY_KI, SWERVE_AUTO_XY_KD), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                    new PIDConstants(SWERVE_AUTO_Z_KP, SWERVE_AUTO_Z_KI, SWERVE_AUTO_Z_KD), // PID constants to correct for rotation error (used to create the rotation controller)
                    swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
                    eventMap,
                    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
            );
            final Command fullAuto = autoBuilder.fullAuto(pathGroup);
            if(pathChooser.getSelected()=="RED_LONG_OUT_AND_BALANCE"||pathChooser.getSelected()=="RED_SHORT_OUT_AND_BALANCE"||pathChooser.getSelected()=="BLUE_LONG_OUT_AND_BALANCE"||pathChooser.getSelected()=="BLUE_SHORT_OUT_AND_BALANCE"){
                return new SequentialCommandGroup(
                        fullAuto.withTimeout(8),
                        new AutoBalanceCommand(swerveSubsystem,false, false)
                );
            }else{return fullAuto;}
        }
    }
}
