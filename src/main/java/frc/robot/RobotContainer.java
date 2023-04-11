// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Autos.AutoBalanceCommand;
import frc.robot.subsystems.*;

import java.util.*;

import static frc.robot.Constants.*;


public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final GrabSubsystem grabSubsystem = new GrabSubsystem();
    private final LadderSubsystem ladderSubsystem = new LadderSubsystem(grabSubsystem);
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final static XboxController controller_driveX = new XboxController(0);
    private final static XboxController controller_Operator = new XboxController(1);
    //private final static XboxController controller_Tester = new XboxController(3);
    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private boolean isStartRecord = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(
                swerveSubsystem,
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kLeftX.value),
                () -> -controller_driveX.getRawAxis(XboxController.Axis.kRightX.value),
                controller_driveX::getPOV,
                controller_driveX::getBackButton,
                controller_driveX::getLeftBumper
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
        pathChooser.setDefaultOption("ONLY OUT", "ONLY_OUT");
        pathChooser.addOption("ONLY BALANCE", "ONLY_BALANCE");
        pathChooser.addOption("MIDDLE OUT AND BALANCE", "MIDDLE_OUT_AND_BALANCE");
        pathChooser.addOption("LONG OUT AND BALANCE", "LONG_OUT_AND_BALANCE");
        pathChooser.addOption("SHORT OUT AND BALANCE", "SHORT_OUT_AND_BALANCE");
        pathChooser.addOption("DONT MOVE", "DONT_MOVE");
        pathChooser.addOption("JUST PUT CUBE", "JUST_PUT_CUBE");
        pathChooser.addOption("PUT CUBE AND BALANCE", "PUT_CUBE_AND_BALANCE");
        pathChooser.addOption("PUT CUBE AND MIDDLE OUT", "PUT_CUBE_AND_MIDDLE_OUT");
        pathChooser.addOption("PUT CUBE AND LONG OUT", "PUT_CUBE_AND_LONG_OUT");
        pathChooser.addOption("PUT CUBE AND SHORT OUT", "PUT_CUBE_AND_SHORT_OUT");
        pathChooser.addOption("PUT CUBE AND MIDDLE OUT THEN BALANCE", "PUT_CUBE_AND_MIDDLE_OUT_AND_BALANCE");
        pathChooser.addOption("PUT CUBE AND LONG OUT THEN BALANCE", "PUT_CUBE_AND_LONG_OUT_AND_BALANCE");
        pathChooser.addOption("PUT CUBE AND SHORT OUT THEN BALANCE", "PUT_CUBE_AND_SHORT_OUT_AND_BALANCE");
        pathChooser.addOption("PUT LOW CUBE AND OUT", "PUT_LOW_CUBE_AND_OUT");
        pathChooser.addOption("PUT LOW CUBE AND BALANCE", "PUT_LOW_CUBE_AND_BALANCE");
        SmartDashboard.putData("Auto choices", pathChooser);
    }


    private void configureBindings()
    {
        new JoystickButton(controller_driveX, XboxController.Button.kRightBumper.value)
                .onTrue(new ParallelCommandGroup(new InstantCommand(swerveSubsystem::zeroGyro),new InstantCommand(()-> swerveSubsystem.setNavXAngle(0))));
        new JoystickButton(controller_Operator, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(grabSubsystem::set_Grabing));
        new JoystickButton(controller_driveX, XboxController.Button.kStart.value)
                .onTrue(new InstantCommand(swerveSubsystem::setBrakingForCharge));
        new JoystickButton(controller_Operator, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(ledSubsystem::setConeColor));
        new JoystickButton(controller_Operator, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(ledSubsystem::setCubeColor));
    }

    public void robotInit() {
    }
    public void disabledPeriodic() {ledSubsystem.rainbow();}
    public void robotPeriodic()
    {
        if(!isStartRecord&&DriverStation.isEnabled()){Shuffleboard.startRecording();}
        if(isStartRecord&&DriverStation.isDisabled()){
            Shuffleboard.stopRecording();
            isStartRecord = false;
        }
    }
    public Command getAutonomousCommand() {
        swerveSubsystem.zeroGyro();
        swerveSubsystem.setNavXAngle(180);
        Command autoPut = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> grabSubsystem.grab = false),
                        new InstantCommand(() -> ladderSubsystem.setLadderLength(5.9)),
                        new InstantCommand(() -> grabSubsystem.setGrabAngle(103.785))
                ),
                new DelayCommand(1.675),
                new InstantCommand(() -> grabSubsystem.grab = true),
                new DelayCommand(1),
                new ParallelCommandGroup(
                        new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
                ),
                new DelayCommand(1),
                new InstantCommand(() -> grabSubsystem.setGrabAngle(20))
        );
        Command autoPutLOW = new SequentialCommandGroup(
                new InstantCommand(() -> grabSubsystem.setGrabAngle(90)),
                new DelayCommand(1.5),
                new InstantCommand(() -> grabSubsystem.grab = true),
                new DelayCommand(1),
                new InstantCommand(() -> grabSubsystem.grab = false),
                new DelayCommand(0.5)
        );
        String autoChosen = pathChooser.getSelected();
        switch (autoChosen) {
            case "DONT_MOVE":
                return null;
            case "ONLY_BALANCE":
                return new AutoBalanceCommand(swerveSubsystem, true, false);
            case "JUST_PUT_CUBE":
                return autoPut;
            case "PUT_CUBE_AND_BALANCE":
                return new SequentialCommandGroup(
                        autoPut,
                        new AutoBalanceCommand(swerveSubsystem, true, false));
            case "PUT_LOW_CUBE_AND_BALANCE":
                return new SequentialCommandGroup(
                        autoPutLOW,
                        new AutoBalanceCommand(swerveSubsystem, true, false));
            default:
                List<String> middleOutAndBalance = Arrays.asList("PUT_CUBE_AND_MIDDLE_OUT_AND_BALANCE", "PUT_CUBE_AND_MIDDLE_OUT", "MIDDLE_OUT_AND_BALANCE");
                List<PathPlannerTrajectory> pathGroup;
                if (middleOutAndBalance.contains(autoChosen)) {
                    pathGroup =
                            PathPlanner.loadPathGroup(pathChooser.getSelected(), new PathConstraints(1, 3));
                } else{
                    pathGroup =
                            PathPlanner.loadPathGroup(pathChooser.getSelected(), new PathConstraints(4, 3));}
                // This is just an example event map. It would be better to have a constant, global event map
                // in your code that will be used by all path following commands
                HashMap<String, Command> eventMap = new HashMap<>();
                // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                        swerveSubsystem::getPose, // Pose2d supplier
                        swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                        swerveDriveKinematics, // SwerveDriveKinematics
                        new PIDConstants(SWERVE_AUTO_XY_KP, SWERVE_AUTO_XY_KI, SWERVE_AUTO_XY_KD), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                        new PIDConstants(SWERVE_AUTO_Z_KP, SWERVE_AUTO_Z_KI, SWERVE_AUTO_Z_KD), // PID constants to correct for rotation error (used to create the rotation controller)
                        swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
                        eventMap,
                        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                        swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
                );
                final Command fullAuto = autoBuilder.fullAuto(pathGroup);
                List<String> putGO = Arrays.asList("PUT_CUBE_AND_LONG_OUT", "PUT_CUBE_AND_SHORT_OUT");
                List<String> putGOBalance = Arrays.asList( "PUT_CUBE_AND_LONG_OUT_AND_BALANCE", "PUT_CUBE_AND_SHORT_OUT_AND_BALANCE");
                List<String> GOBalance = Arrays.asList( "LONG_OUT_AND_BALANCE", "SHORT_OUT_AND_BALANCE", "PUT_CUBE_AND_BALANCE");
                if (putGO.contains(autoChosen)) {
                    return new SequentialCommandGroup
                            (autoPut, fullAuto.withTimeout(4.25));
                } else if (autoChosen=="PUT_CUBE_AND_MIDDLE_OUT") {
                    return new SequentialCommandGroup(
                            autoPut, fullAuto.withTimeout(7),
                            new AutoBalanceCommand(swerveSubsystem, false, false));
                } else if (autoChosen=="MIDDLE_OUT_AND_BALANCE") {
                    return new SequentialCommandGroup(fullAuto.withTimeout(7), new AutoBalanceCommand(swerveSubsystem, false, false));
                } else if(autoChosen=="PUT_CUBE_AND_MIDDLE_OUT_AND_BALANCE"){
                    return new SequentialCommandGroup(
                            autoPut, fullAuto.withTimeout(7),
                            new AutoBalanceCommand(swerveSubsystem, false, false));
                } else if (autoChosen=="PUT_LOW_CUBE_AND_OUT") {
                    return  new SequentialCommandGroup(
                            autoPutLOW,fullAuto.withTimeout(10)
                    );
                } else if (putGOBalance.contains(autoChosen)) {
                    return new SequentialCommandGroup(
                            autoPut, fullAuto.withTimeout(4.25),
                            new AutoBalanceCommand(swerveSubsystem, false, false));
                } else if (GOBalance.contains(autoChosen)) {
                    return new SequentialCommandGroup(
                            fullAuto.withTimeout(4.25),
                            new AutoBalanceCommand(swerveSubsystem, false, false));
                } else {return fullAuto;}
        }
    }
}