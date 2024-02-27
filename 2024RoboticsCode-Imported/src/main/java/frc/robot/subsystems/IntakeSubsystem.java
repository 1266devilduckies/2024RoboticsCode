package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeSpin;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakeSpinState {
        TAKE_IN,
        SHOOT_OUT,
        STOPPED;
    }
    
    VictorSPX intakeSpinMotor = new VictorSPX(Constants.Intake.spinID);
    TalonSRX intakeSpinMotorFollower = new TalonSRX(Constants.Intake.spinIDFollower);

    IntakeSpinState intakeSpinState = IntakeSpinState.STOPPED;

    RobotContainer robotContainer;

    public IntakeSubsystem(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        configure();
    }

    private void configure(){
        intakeSpinMotor.setNeutralMode(NeutralMode.Coast);
        intakeSpinMotorFollower.setNeutralMode(NeutralMode.Coast);

        intakeSpinMotorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 25, 0.5));

        intakeSpinMotorFollower.follow(intakeSpinMotor);
    }

    @Override
    public void periodic() {
        //set the spinMotor to whatever intakeSpinState is
        switch(intakeSpinState){
            case STOPPED:
                intakeSpinMotorFollower.set(ControlMode.PercentOutput, Constants.Intake.intakeStoppedSpeed);
                SmartDashboard.putString("SpinState", "STOPPED");
                break;
            case TAKE_IN:
                intakeSpinMotorFollower.set(ControlMode.PercentOutput, Constants.Intake.intakeTakeInSpeed);
                SmartDashboard.putString("SpinState", "TAKE_IN");
                break;
            case SHOOT_OUT:
                intakeSpinMotorFollower.set(ControlMode.PercentOutput, Constants.Intake.intakeShootOutSpeed);
                SmartDashboard.putString("SpinState", "SHOOT_OUT");
        }

        /*
        // check for holding note, then pull it back a bit
        if(robotContainer.getLaunchSubsystem().checkForNote()){
            new SequentialCommandGroup(
                new IntakeSpin(this, IntakeSpinState.SHOOT_OUT), 
                new WaitCommand(0.1),
                new IntakeSpin(this, IntakeSpinState.STOPPED)).schedule();
        }*/
    }

    public boolean currentSense(){
        if(intakeSpinMotorFollower.getStatorCurrent() > 35){
            return true;
        }
        return false;
    }

    public void setSpinState(IntakeSpinState state){
        this.intakeSpinState = state;
    }

}