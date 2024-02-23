package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LaunchSubsystem extends SubsystemBase {
    
    TalonFX launchMotor = new TalonFX(Constants.Launch.launchMotorID);
    TalonFX launchMotorFollower = new TalonFX(Constants.Launch.launchMotorFollowerID);

    TalonFXConfiguration launchMotorConfig;
    TalonFXConfiguration launchMotorFollowerConfig;

    private boolean holdingNote = false;
    private double lastTickPosition = 0;

    public LaunchSubsystem(RobotContainer robotContainer){
        //config motors
        launchMotorConfig.Slot0.kP = Constants.Launch.launchP;
        launchMotorConfig.Slot0.kI = Constants.Launch.launchI;
        launchMotorConfig.Slot0.kD = Constants.Launch.launchD;

        launchMotorFollowerConfig.Slot0.kP = Constants.Launch.launchP;
        launchMotorFollowerConfig.Slot0.kI = Constants.Launch.launchI;
        launchMotorFollowerConfig.Slot0.kD = Constants.Launch.launchD;

        launchMotor.getConfigurator().apply(launchMotorConfig);
        launchMotorFollower.getConfigurator().apply(launchMotorFollowerConfig);

        launchMotor.setNeutralMode(NeutralModeValue.Coast);
        launchMotorFollower.setNeutralMode(NeutralModeValue.Coast);

        launchMotorFollower.setControl(new Follower(launchMotor.getDeviceID(), false));
    }

    @Override
    public void periodic() {
        
    }

    public void setLaunchMotorVelocity(int velocity){
        launchMotor.setControl(new VelocityDutyCycle(velocity));
    }

    public void stopLaunchMotors(){
        launchMotor.set(0);

        lastTickPosition = launchMotor.getPosition().getValueAsDouble();
    }

    public void setHoldingNote(boolean holding){
        this.holdingNote = holding;
    }

    public boolean checkForNote(){
        // if not holding note
        if(!holdingNote){
            // if difference from last encoder location to current is greater then a small threshold
            if(Math.abs(lastTickPosition - launchMotor.getPosition().getValueAsDouble()) > 0.1){
                // return true, set holding note to true
                setHoldingNote(false);
                return true;
            }
        }
        return false;
    }

}
