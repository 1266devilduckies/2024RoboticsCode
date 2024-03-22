package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    
    TalonFX intakeSpinMotor = new TalonFX(Constants.Intake.spinID);

    TalonFXConfiguration intakeMotorConfiguration;

    IntakeSpinState intakeSpinState = IntakeSpinState.STOPPED;

    RobotContainer robotContainer;

    public IntakeSubsystem(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        this.intakeMotorConfiguration = new TalonFXConfiguration();
        configure();
    }

    private void configure(){
        intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 35;

        intakeSpinMotor.getConfigurator().apply(intakeMotorConfiguration);
    }

    @Override
    public void periodic() {
        //set the spinMotor to whatever intakeSpinState is
        switch(intakeSpinState){
            case STOPPED:
                intakeSpinMotor.setControl(new DutyCycleOut(Constants.Intake.intakeStoppedSpeed));
                break;
            case TAKE_IN:
                intakeSpinMotor.setControl(new DutyCycleOut(Constants.Intake.intakeTakeInSpeed));
                break;
            case SHOOT_OUT:
                intakeSpinMotor.setControl(new DutyCycleOut(Constants.Intake.intakeStoppedSpeed));
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
        if(intakeSpinMotor.getStatorCurrent().getValueAsDouble() > 25.0){
            return true;
        }
        return false;
    }

    public void setSpinState(IntakeSpinState state){
        this.intakeSpinState = state;
    }

    public IntakeSpinState getIntakeSpinState(){
        return intakeSpinState;
    }
}