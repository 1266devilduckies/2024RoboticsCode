package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LaunchSubsystem extends SubsystemBase {
    
    TalonFX launchMotor = new TalonFX(Constants.Launch.launchMotorID);
    TalonFX launchMotorFollower = new TalonFX(Constants.Launch.launchMotorFollowerID);

    TalonFXConfiguration launchMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration launchMotorFollowerConfig = new TalonFXConfiguration();

    SlewRateLimiter launchMotorLimiter = new SlewRateLimiter(0.5);

    private boolean holdingNote = true;
    private double lastTickPosition = 0;

    public LaunchSubsystem(RobotContainer robotContainer){
        //config motors
        launchMotorConfig.Slot0.kP = Constants.Launch.launchP;
        launchMotorConfig.Slot0.kI = Constants.Launch.launchI;
        launchMotorConfig.Slot0.kD = Constants.Launch.launchD;

        launchMotorFollowerConfig.Slot0.kP = Constants.Launch.launchP;
        launchMotorFollowerConfig.Slot0.kI = Constants.Launch.launchI;
        launchMotorFollowerConfig.Slot0.kD = Constants.Launch.launchD;

        launchMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorConfig.CurrentLimits.StatorCurrentLimit = 30;

        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimit = 30;

        launchMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        launchMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;

        launchMotor.getConfigurator().apply(launchMotorConfig);
        launchMotorFollower.getConfigurator().apply(launchMotorFollowerConfig);

        launchMotor.setNeutralMode(NeutralModeValue.Coast);
        launchMotorFollower.setNeutralMode(NeutralModeValue.Coast);

    }

    @Override
    public void periodic() {
        
    }

    public void setLaunchMotorSpeed(double speed){
        launchMotor.set(speed);
        launchMotorFollower.set(speed);
    }

    public void stopLaunchMotors(){
        launchMotor.set(0);
        launchMotorFollower.set(0);
    }

    public void setHoldingNote(boolean holding){
        this.holdingNote = holding;
    }

}
