package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
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

    private boolean holdingNote = true;
    private double lastTickPosition = 0;

    final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);

    public LaunchSubsystem(RobotContainer robotContainer){
        launchMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorConfig.CurrentLimits.StatorCurrentLimit = 35;

        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimit = 35;

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
        launchMotor.setControl(dutyCycleOut.withOutput(speed));
        launchMotorFollower.setControl(dutyCycleOut.withOutput(speed));
    }

    public void stopLaunchMotors(){
        launchMotor.set(0);
        launchMotorFollower.set(0);
    }

    public void setHoldingNote(boolean holding){
        this.holdingNote = holding;
    }

    public double getMotorPosition(){
        return launchMotor.getPosition().getValueAsDouble();
    }

}
