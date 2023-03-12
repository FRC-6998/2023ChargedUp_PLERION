package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GrabSubsystem;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;

public class AutoPutCommand extends CommandBase {

    public AutoPutCommand(String gamePieceType, int floor,
                          LadderSubsystem ladderSubsystem, GrabSubsystem grabSubsystem){
        if(gamePieceType=="CUBE"){
            switch (floor) {
                case 1:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(0)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(90))
                            ),
                            new DelayCommand(0.5),
                            new InstantCommand(() -> grabSubsystem.grab = false)
                    );
                    break;
                case 2:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CUBE_SECOND_LENGTH)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CUBE_SECOND_ANGLE))
                            ),
                            new DelayCommand(1),
                            new InstantCommand(() -> grabSubsystem.grab = false),
                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
                    );
                    break;
                case 3:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CUBE_THIRD_LENGTH)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CUBE_THIRD_ANGLE))
                            ),
                            new DelayCommand(1.375),
                            new InstantCommand(() -> grabSubsystem.grab = false),
                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
                    );
                    break;
                default:
                    break;
            }
        } else if (gamePieceType=="CONE") {
            switch (floor) {
                case 1:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(0)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(90))
                            ),
                            new DelayCommand(0.5),
                            new InstantCommand(() -> grabSubsystem.grab = false)
                    );
                    break;
                case 2:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CONE_SECOND_LENGTH)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CONE_SECOND_ANGLE))
                            ),
                            new DelayCommand(1),
                            new InstantCommand(() -> grabSubsystem.grab = false),
                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
                    );
                    break;
                case 3:
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CONE_THIRD_LENGTH)),
                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CONE_THIRD_ANGLE))
                            ),
                            new DelayCommand(1.375),
                            new InstantCommand(() -> grabSubsystem.grab = false),
                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
                    );
                    break;
                default:
                    break;
            }
        }
    }
}

