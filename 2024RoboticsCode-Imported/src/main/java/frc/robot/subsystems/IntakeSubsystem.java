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

import edu.wpi.first.wpilibj.DigitalInput;
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
    TalonFX intakeSpinFollower = new TalonFX(Constants.Intake.spinIDFollower);

    TalonFXConfiguration intakeMotorConfiguration;
    TalonFXConfiguration intakeFollowerConfiguration;

    IntakeSpinState intakeSpinState = IntakeSpinState.STOPPED;

    DigitalInput beambreak = new DigitalInput(Constants.Intake.beambreakDIO);

    RobotContainer robotContainer;

    public IntakeSubsystem(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
        this.intakeMotorConfiguration = new TalonFXConfiguration();
        this.intakeFollowerConfiguration = new TalonFXConfiguration();
        configure();
    }

    private void configure(){
        intakeSpinMotor.setNeutralMode(NeutralModeValue.Coast);
        intakeSpinFollower.setNeutralMode(NeutralModeValue.Coast);

        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 35;

        intakeFollowerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeFollowerConfiguration.CurrentLimits.SupplyCurrentLimit = 35;

        intakeSpinFollower.setInverted(false);

        intakeSpinMotor.getConfigurator().apply(intakeMotorConfiguration);
        intakeSpinFollower.getConfigurator().apply(intakeFollowerConfiguration);
    }

    @Override
    public void periodic() {
        //set the spinMotor to whatever intakeSpinState is
        switch(intakeSpinState){
            case STOPPED:
                intakeSpinMotor.set(Constants.Intake.intakeStoppedSpeed);
                //intakeSpinFollower.set(Constants.Intake.intakeStoppedSpeed);
                break;
            case TAKE_IN:
                intakeSpinMotor.set(Constants.Intake.intakeTakeInSpeed);
                //intakeSpinFollower.set(Constants.Intake.intakeTakeInSpeed);
                break;
            case SHOOT_OUT:
                intakeSpinMotor.set(Constants.Intake.intakeShootOutSpeed);
                //intakeSpinFollower.set(Constants.Intake.intakeShootOutSpeed);
        }

        // check if holding note


        /*
        // check for holding note, then pull it back a bit
        if(robotContainer.getLaunchSubsystem().checkForNote()){
            new SequentialCommandGroup(
                new IntakeSpin(this, IntakeSpinState.SHOOT_OUT), 
                new WaitCommand(0.1),
                new IntakeSpin(this, IntakeSpinState.STOPPED)).schedule();
        }*/
    }

    public boolean holdingNote(){
        return !beambreak.get();
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