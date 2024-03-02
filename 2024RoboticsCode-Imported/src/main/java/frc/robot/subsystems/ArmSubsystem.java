package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase{
    
    public enum ArmState{
        // This is where you add new arm position and set there angle in degrees, taken from Shuffleboard
        NEUTRAL(140),
        SHOOT_SPEAKER_FRONT(192),
        GROUND_PICKUP(95),
        SHOOT_SPEAKER_SIDE_CORNER(192);

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

    WPI_TalonSRX armEncoder = new WPI_TalonSRX(Constants.Arm.encoderID);

    TalonFX armMotor = new TalonFX(Constants.Arm.armID);
    TalonFX armMotorFollower = new TalonFX(Constants.Arm.armFollowerID);

    TalonFXConfiguration armMotorConfiguration;
    TalonFXConfiguration armMotorFollowerConfiguration;

    ArmState currentArmState = null;

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

        armMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        armMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30;

        armMotorFollowerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        armMotorFollowerConfiguration.CurrentLimits.StatorCurrentLimit = 30;

        armMotor.getConfigurator().apply(armMotorConfiguration);
        armMotorFollower.getConfigurator().apply(armMotorFollowerConfiguration);

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotorFollower.setNeutralMode(NeutralModeValue.Brake);

        armMotor.setInverted(true);
        armMotorFollower.setInverted(true);

        armMotorFollower.setControl(new Follower(armMotor.getDeviceID(), true));
    }

    @Override
    public void periodic(){
        //DEBUG
        if(RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.elbowMovementAxis) > 0.05){
            armMotor.setVoltage(2.5);
        }
        else if(RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.elbowMovementAxis) < -0.05){
            armMotor.setVoltage(-2.5);
        }
        else{
            armMotor.setVoltage(0);
        }

        SmartDashboard.putNumber("Arm Angle", getArmAngle());

        if(currentArmState == null) return;
        setArmAngle(currentArmState.getAngle());
    }

    public void setArmState(ArmState state){
        this.currentArmState = state;
    }

    public double getArmAngle(){
        // 1 degree = 11.37
        return armEncoder.getSensorCollection().getPulseWidthRiseToFallUs() / 11.37;
    }

    private void setArmAngle(double angle){
        double feedForward = armFeedforward.calculate(getArmAngle(), angle);
        //voltage is negative due to that being the "forward direction"
        double clampedVoltage = MathUtil.clamp(pidController.calculate(getArmAngle(), angle), -2, 2);
        armMotor.setVoltage(clampedVoltage);
    }

    public boolean atSetpoint(){
        if(Math.abs(getArmAngle() - currentArmState.getAngle()) < 5){
            return true;
        }
        return false;
    }

}