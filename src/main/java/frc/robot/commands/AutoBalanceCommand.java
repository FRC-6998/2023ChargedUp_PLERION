package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;


public class AutoBalanceCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final PIDController pidController = new PIDController(
            Constants.AUTO_BALANCE_KP,Constants.AUTO_BALANCE_KI,Constants.AUTO_BALANCE_KD);
    private final Timer timer = new Timer();
    private Stage currentStage;
    private final boolean shouldExitWhenFinished;

    public AutoBalanceCommand(boolean shouldExitWhenFinished) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);
        currentStage = Stage.Created;
        this.shouldExitWhenFinished = shouldExitWhenFinished;
    }

    @Override
    public void initialize() {
        currentStage = Stage.Preparing;
        timer.start();
    }

    @Override
    public void execute() {
        double error = swerveSubsystem.navX.getPitch();
        switch (currentStage){
            case Preparing:
                swerveSubsystem.drive(new Translation2d(0.1, 0).times(SWERVE_MAX_SPEED), 0, false, false);
                if(Math.abs(error)>Constants.AUTO_BALANCE_TOLERANCE){
                    currentStage = Stage.Climb;
                    timer.reset();
                }
                break;
            case Climb:
                swerveSubsystem.drive(new Translation2d(0.1, 0).times(SWERVE_MAX_SPEED),0,false,false);
                if(Math.abs(error)<=Constants.AUTO_BALANCE_TOLERANCE){
                    currentStage = Stage.Wait;
                    timer.reset();
                }
                break;
            case Wait:
                if(Math.abs(error)>Constants.AUTO_BALANCE_TOLERANCE){
                    currentStage = Stage.Climb;
                }else{
                    if(timer.get()>Constants.AUTO_BALANCE_WAIT_TIME){
                        currentStage = Stage.Finish;
                    }
                }
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return shouldExitWhenFinished && currentStage==Stage.Finish;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(
                new Translation2d(0, 0).times(SWERVE_MAX_SPEED),
                0,false,false);
        timer.stop();
    }

    public enum Stage{
        Created,Preparing,Climb,Wait,Finish
    }
}
