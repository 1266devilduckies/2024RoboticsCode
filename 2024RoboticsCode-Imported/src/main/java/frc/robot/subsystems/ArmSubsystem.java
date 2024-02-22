package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase{
    
    public enum ArmState{
        INITIAL(10),
        SHOOT_SPEAKER_FRONT(20),
        GROUND_PICKUP(30),
        SHOOT_SPEAKER_SIDE_CORNER(20);

        private double angle;

        private ArmState(double angle){
            this.angle = angle;
        }

        private double getAngle(){
            return angle;
        }
    }

    PIDController armPidController = new PIDController(
        Constants.Arm.armP, Constants.Arm.armI, Constants.Arm.armD);
    
    TalonSRX armEncoder = new TalonSRX(Constants.Arm.encoderID);

    TalonFX armMotor = new TalonFX(Constants.Arm.armID);
    TalonFX armMotorFollower = new TalonFX(Constants.Arm.armFollowerID);

    TalonFXConfiguration armMotorConfiguration;
    TalonFXConfiguration armMotorFollowerConfiguration;

    ArmState currentArmState = ArmState.INITIAL;

    public ArmSubsystem(RobotContainer robotContainer){
        armMotorConfiguration = new TalonFXConfiguration();
        armMotorFollowerConfiguration = new TalonFXConfiguration();        
        
        configure();
    }

    private void configure(){
        armMotorConfiguration.Slot0.kP = Constants.Arm.armDefaultP;
        armMotorConfiguration.Slot0.kI = Constants.Arm.armDefaultI;
        armMotorConfiguration.Slot0.kD = Constants.Arm.armDefaultD;

        armMotorFollowerConfiguration.Slot0.kP = Constants.Arm.armDefaultP;
        armMotorFollowerConfiguration.Slot0.kI = Constants.Arm.armDefaultI;
        armMotorFollowerConfiguration.Slot0.kD = Constants.Arm.armDefaultD;

        armMotor.getConfigurator().apply(armMotorConfiguration);
        armMotorFollower.getConfigurator().apply(armMotorFollowerConfiguration);

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotorFollower.setNeutralMode(NeutralModeValue.Brake);

        armMotorFollower.setControl(new Follower(armMotor.getDeviceID(), false));
    }

    @Override
    public void periodic(){
        setArmAngle(currentArmState.getAngle());
    }

    public void setArmState(ArmState state){
        this.currentArmState = state;
    }

    public double getArmAngle(){
        // convert the sensor units into angle?
        return armEncoder.getSelectedSensorPosition();
    }

    private void setArmAngle(double angle){
        armMotor.set(armPidController.calculate(getArmAngle(), angle));
    }

}
