package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;

public class Shoot extends Command {
    
    private LaunchSubsystem launchSubsystem;

    private int velocity;

    private double startTime = 0;
    private final double timeLimit = 2;

    public Shoot(LaunchSubsystem subsystem, int velocity){
        this.launchSubsystem = subsystem;
        this.velocity = velocity;
        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();

        launchSubsystem.setLaunchMotorVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted){
        launchSubsystem.stopLaunchMotors();
    }

    @Override
    public boolean isFinished(){
        return (startTime + timeLimit > Timer.getFPGATimestamp());
    }

}
