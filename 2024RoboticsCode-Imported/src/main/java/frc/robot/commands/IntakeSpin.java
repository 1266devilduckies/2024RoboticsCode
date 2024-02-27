package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinState;

public class IntakeSpin extends Command{
    private final IntakeSubsystem intakeSubsystem;

    private IntakeSpinState state;

    double timeStart = 0;
    double timeWait = 1.5;

    boolean rising = false;

    public IntakeSpin(IntakeSubsystem subsystem, IntakeSpinState intakeState, boolean rising){
        this.intakeSubsystem = subsystem;
        this.state = intakeState;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setSpinState(state);
        timeStart = Timer.getFPGATimestamp();

        // get current drivetrain wheel position
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setSpinState(IntakeSpinState.STOPPED);
    }

    @Override
    public boolean isFinished(){
        if(rising){
            return ((timeStart + timeWait) < Timer.getFPGATimestamp());
        }
        else{
            return intakeSubsystem.currentSense();
        }
    }
}
