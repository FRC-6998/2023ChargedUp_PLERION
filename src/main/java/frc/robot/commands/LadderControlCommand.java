package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LadderSubsystem;
import java.util.function.IntSupplier;

import static frc.robot.Constants.*;

public class LadderControlCommand extends CommandBase {
    private final LadderSubsystem ladderSubsystem;
    private final IntSupplier povLadderSup;
    private final Timer timer = new Timer();
    private double ladderLength = 0;

    public LadderControlCommand(LadderSubsystem ladderSubsystem, IntSupplier povLadderSup) {
        this.ladderSubsystem = ladderSubsystem;
        this.povLadderSup = povLadderSup;

        addRequirements(ladderSubsystem);
    }
    public void zeroLadderLength(){ladderLength=0;}

    @Override
    public void execute() {
        switch (povLadderSup.getAsInt()){
            case 0:
                ladderLength+=GRAB_ANGLECONTROL_NUM;
                timer.start();
                if(timer.get()>=LADDER_POVCONTROL_WAITTIME&&ladderLength<=LADDER_MAX_LENGTH){
                    ladderLength+=GRAB_ANGLECONTROL_NUM;
                    timer.restart();
                }
                break;
            case 180:
                ladderLength-=GRAB_ANGLECONTROL_NUM;
                timer.start();
                if(timer.get()>=LADDER_POVCONTROL_WAITTIME&&ladderLength>=0){
                    ladderLength-=GRAB_ANGLECONTROL_NUM;
                    timer.restart();
                }
                break;
            default:
                timer.reset();
                break;
        }
        ladderSubsystem.setLadderLength(ladderLength);
    }
}