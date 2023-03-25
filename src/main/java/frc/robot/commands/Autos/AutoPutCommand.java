package frc.robot.commands.Autos;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DelayCommand;
import frc.robot.subsystems.GrabSubsystem;
import frc.robot.subsystems.LadderSubsystem;

import static frc.robot.Constants.*;

public class AutoPutCommand extends CommandBase {
    public AutoPutCommand(LadderSubsystem ladderSubsystem, GrabSubsystem grabSubsystem){
        new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> ladderSubsystem.setLadderLength(4)),
                        new InstantCommand(() -> grabSubsystem.setGrabAngle(90))
                ),
                new DelayCommand(2),
                new InstantCommand(() -> grabSubsystem.grab = false)
        );
    }

    //Complex AutoPutCommand
//    public AutoPutCommand(String gamePieceType, int floor,
//                          LadderSubsystem ladderSubsystem, GrabSubsystem grabSubsystem){
//        if(gamePieceType=="CUBE"){
//            switch (floor) {
//                case 1:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(0)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(90))
//                            ),
//                            new DelayCommand(0.5),
//                            new InstantCommand(() -> grabSubsystem.grab = false)
//                    );
//                    break;
//                case 2:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CUBE_AUTO_SECOND_LENGTH)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CUBE_AUTO_SECOND_ANGLE))
//                            ),
//                            new DelayCommand(1),
//                            new InstantCommand(() -> grabSubsystem.grab = false),
//                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
//                    );
//                    break;
//                case 3:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CUBE_AUTO_THIRD_LENGTH)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CUBE_AUTO_THIRD_ANGLE))
//                            ),
//                            new DelayCommand(1.375),
//                            new InstantCommand(() -> grabSubsystem.grab = false),
//                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
//                    );
//                    break;
//                default:
//                    break;
//            }
//        } else if (gamePieceType=="CONE") {
//            switch (floor) {
//                case 1:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(0)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(90))
//                            ),
//                            new DelayCommand(0.5),
//                            new InstantCommand(() -> grabSubsystem.grab = false)
//                    );
//                    break;
//                case 2:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CONE_AUTO_SECOND_LENGTH)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CONE_AUTO_SECOND_ANGLE))
//                            ),
//                            new DelayCommand(1),
//                            new InstantCommand(() -> grabSubsystem.grab = false),
//                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
//                    );
//                    break;
//                case 3:
//                    new SequentialCommandGroup(
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> ladderSubsystem.setLadderLength(CONE_AUTO_THIRD_LENGTH)),
//                                    new InstantCommand(() -> grabSubsystem.setGrabAngle(CONE_AUTO_THIRD_ANGLE))
//                            ),
//                            new DelayCommand(1.375),
//                            new InstantCommand(() -> grabSubsystem.grab = false),
//                            new InstantCommand(() -> ladderSubsystem.setLadderLength(0))
//                    );
//                    break;
//                default:
//                    break;
//            }
//        }
//    }
}

