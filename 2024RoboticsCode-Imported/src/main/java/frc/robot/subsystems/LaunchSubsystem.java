package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LaunchSubsystem extends SubsystemBase {

    VelocityVoltage velocityRequest;
    double leftFF, rightFF;
    
    TalonFX launchMotor = new TalonFX(Constants.Launch.launchMotorID);
    TalonFX launchMotorFollower = new TalonFX(Constants.Launch.launchMotorFollowerID);

    TalonFXConfiguration launchMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration launchMotorFollowerConfig = new TalonFXConfiguration();

    private boolean holdingNote = true;

    /**
     * <b> Units: </b>
     * Rotations per second
     */
    double desiredLeftVelocity = 0;
    /**
     * <b> Units: </b>
     * Rotations per second
     */
    double desiredRightVelocity = 0;

    public LaunchSubsystem(RobotContainer robotContainer){

        velocityRequest = new VelocityVoltage(0).withSlot(0);

        launchMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorConfig.CurrentLimits.StatorCurrentLimit = 50;

        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        launchMotorFollowerConfig.CurrentLimits.StatorCurrentLimit = 60;

        launchMotorConfig.Slot0.kP = Constants.Launch.launchP;
        launchMotorConfig.Slot0.kI = Constants.Launch.launchI;
        launchMotorConfig.Slot0.kD = Constants.Launch.launchD;
    
        launchMotorFollowerConfig.Slot0.kP = Constants.Launch.launchPFollower;
        launchMotorFollowerConfig.Slot0.kI = Constants.Launch.launchIFollower;
        launchMotorFollowerConfig.Slot0.kD = Constants.Launch.launchDFollower;

        launchMotor.getConfigurator().apply(launchMotorConfig);
        launchMotorFollower.getConfigurator().apply(launchMotorFollowerConfig);

        launchMotor.setNeutralMode(NeutralModeValue.Coast);
        launchMotorFollower.setNeutralMode(NeutralModeValue.Coast);

        leftFF = -1;
        rightFF = -2;
    }

    @Override
    public void periodic() {
        
    }

    public void setLaunchMotorSpeed(double desiredVelocity_RPS){
        launchMotor.setControl(velocityRequest.withVelocity(desiredVelocity_RPS).withFeedForward(leftFF).withAcceleration(-30));
        launchMotorFollower.setControl(velocityRequest.withVelocity(desiredVelocity_RPS).withFeedForward(rightFF).withAcceleration(-30));
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
