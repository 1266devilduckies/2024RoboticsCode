package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinState;

public class Shoot extends Command {
    
    private LaunchSubsystem launchSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private int velocity;

    private double startTime = 0;
    private final double timeLimit = 2;
    private final double feedTime = 0.5;

    public Shoot(LaunchSubsystem subsystem, IntakeSubsystem intakeSubsystem, int velocity){
        this.launchSubsystem = subsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.velocity = velocity;
        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();

        launchSubsystem.setLaunchMotorVelocity(velocity);
    }

    @Override
    public void execute(){
        if (startTime + feedTime > Timer.getFPGATimestamp()){
            intakeSubsystem.setSpinState(IntakeSpinState.TAKE_IN);
        }
    }

    @Override
    public void end(boolean interrupted){
        launchSubsystem.stopLaunchMotors();
        intakeSubsystem.setSpinState(IntakeSpinState.STOPPED);
        launchSubsystem.setHoldingNote(false);
    }

    @Override
    public boolean isFinished(){
        return ((startTime + timeLimit) > Timer.getFPGATimestamp());
    }

}
