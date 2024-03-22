package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class MoveArm extends Command {
    
    private ArmSubsystem armSubsystem;

    private ArmState state;

    public MoveArm(ArmSubsystem subsystem, ArmState state){
        this.armSubsystem = subsystem;
        this.state = state;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArmState(state);
    }

    @Override
    public boolean isFinished(){
        return armSubsystem.atSetpoint();
    }

}
