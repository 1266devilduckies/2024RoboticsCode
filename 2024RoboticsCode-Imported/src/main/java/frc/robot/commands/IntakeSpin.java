package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinState;

public class IntakeSpin extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final LaunchSubsystem launchSubsystem;

    private IntakeSpinState state;

    double timeStart = 0;
    double timeWait = 0.5;

    boolean rising = false;

    double initPosition = 0;
    double amountNeeded = -0.05;

    public IntakeSpin(IntakeSubsystem subsystem, LaunchSubsystem launchSubsystem, IntakeSpinState intakeState, boolean rising){
        this.intakeSubsystem = subsystem;
        this.launchSubsystem = launchSubsystem;
        this.state = intakeState;
        this.rising = rising;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setSpinState(state);
        timeStart = Timer.getFPGATimestamp();

        initPosition = launchSubsystem.getMotorPosition();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setSpinState(IntakeSpinState.STOPPED);
    }

    @Override
    public boolean isFinished(){
        if(rising){
            return (timeStart + timeWait) < Timer.getFPGATimestamp();
        }
        return (initPosition + amountNeeded) > launchSubsystem.getMotorPosition(); 
    }
}
