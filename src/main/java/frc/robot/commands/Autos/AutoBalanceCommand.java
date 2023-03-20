package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.*;


public class AutoBalanceCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController pidController = new PIDController(
            Constants.AUTO_BALANCE_KP, Constants.AUTO_BALANCE_KI, Constants.AUTO_BALANCE_KD);
    private final Timer timer = new Timer();
    private Stage currentStage;
    private final boolean shouldExitWhenFinished;
    private final boolean startInverted;

    public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, boolean startInverted, boolean shouldExitWhenFinished) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.swerveSubsystem = swerveSubsystem;
        this.shouldExitWhenFinished = shouldExitWhenFinished;
        this.startInverted = startInverted;
        addRequirements(swerveSubsystem);
        currentStage = Stage.Created;
    }

    @Override
    public void initialize() {
        currentStage = Stage.Preparing;
        timer.start();
    }

    @Override
    public void execute() {
        double error = swerveSubsystem.navX.getRoll();
        switch (currentStage){
            case Preparing:
                swerveSubsystem.drive(
                        new Translation2d(startInverted?-AUTO_BALANCE_PREPARING_SPEED:
                                AUTO_BALANCE_PREPARING_SPEED, 0).times(SWERVE_MAX_SPEED),
                        0, false, false);
                if(Math.abs(error)> AUTO_PREPARE_CHANGENUM){
                    currentStage = Stage.Climb;
                    timer.reset();
                }
                break;
            case Climb:
                swerveSubsystem.drive(new Translation2d(pidController.calculate(error), 0).times(SWERVE_MAX_SPEED),0,false,false);
                if(Math.abs(error)<=Constants.AUTO_BALANCE_TOLERANCE){
                    currentStage = Stage.Wait;
                    timer.reset();
                }
                break;
            case Wait:
                swerveSubsystem.drive(new Translation2d(0, 0),0, false, false);
                if(Math.abs(error)>Constants.AUTO_BALANCE_TOLERANCE){
                    currentStage = Stage.Climb;
                }else{
                    if(shouldExitWhenFinished && timer.get()>Constants.AUTO_BALANCE_WAIT_TIME){
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
        Created, Preparing, Climb, Wait, Finish
    }
}
