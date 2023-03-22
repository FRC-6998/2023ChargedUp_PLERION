package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LadderSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.*;

public class LadderControlCommand extends CommandBase {
    private final LadderSubsystem ladderSubsystem;
    private final IntSupplier povLadderSup;
    private final BooleanSupplier zeroButtonSup;
    private final Timer timer = new Timer();
    private double ladderLength = 0;
    private int ladderFloor = 1;
    public LadderControlCommand(
            LadderSubsystem ladderSubsystem, IntSupplier povLadderSup, BooleanSupplier zeroButtonSup) {
        this.ladderSubsystem = ladderSubsystem;
        this.povLadderSup = povLadderSup;
        this.zeroButtonSup = zeroButtonSup;

        addRequirements(ladderSubsystem);
    }

    @Override
    public void execute() {
        if(zeroButtonSup.getAsBoolean()){ladderLength = 0;}
        switch (povLadderSup.getAsInt()){
            case 0:
                if(ladderLength <= LADDER_MAX_LENGTH){ladderLength+=LADDER_POVCONTROL_NUM;}
                timer.start();
                if(timer.get()>=LADDER_POVCONTROL_WAITTIME && ladderLength <= LADDER_MAX_LENGTH){
                    ladderLength+=GRAB_ANGLECONTROL_NUM;
                    timer.restart();
                }
                break;
            case 180:
                if(ladderLength >= 0){ladderLength-=LADDER_POVCONTROL_NUM;}
                timer.start();
                if(timer.get()>=LADDER_POVCONTROL_WAITTIME && ladderLength >= 0){
                    ladderLength-=LADDER_POVCONTROL_NUM;
                    timer.restart();
                }
                break;
            default:
                timer.reset();
                break;
        }
        if(DriverStation.isTeleopEnabled()){ladderSubsystem.setLadderLength(ladderLength);}
        SmartDashboard.putNumber("ladderLength", ladderLength);
    }
}