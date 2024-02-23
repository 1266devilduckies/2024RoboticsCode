package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase{
    
    public enum ArmState{
        // This is where you add new arm position and set there angle in degrees, taken from Shuffleboard
        NEUTRAL(10),
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

    ArmFeedforward armFeedforward = new ArmFeedforward(
        Constants.Arm.armS, 
        Constants.Arm.armG, 
        Constants.Arm.armV
    );

    PIDController pidController = new PIDController(Constants.Arm.armP, 0, 0);

    TalonSRX armEncoder = new TalonSRX(Constants.Arm.encoderID);

    TalonFX armMotor = new TalonFX(Constants.Arm.armID);
    TalonFX armMotorFollower = new TalonFX(Constants.Arm.armFollowerID);

    TalonFXConfiguration armMotorConfiguration;
    TalonFXConfiguration armMotorFollowerConfiguration;

    ArmState currentArmState = null;

    public ArmSubsystem(RobotContainer robotContainer){
        armMotorConfiguration = new TalonFXConfiguration();
        armMotorFollowerConfiguration = new TalonFXConfiguration();    
        
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
        
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
        if(currentArmState == null) return;
        setArmAngle(currentArmState.getAngle());
    }

    public void setArmState(ArmState state){
        this.currentArmState = state;
    }

    public double getArmAngle(){
        // 1 degree = 11.37
        return armEncoder.getSelectedSensorPosition() / 11.37;
    }

    private void setArmAngle(double angle){
        double feedForward = armFeedforward.calculate(getArmAngle(), angle);
        armMotor.setVoltage(pidController.calculate(getArmAngle(), angle) + feedForward);
    }

}