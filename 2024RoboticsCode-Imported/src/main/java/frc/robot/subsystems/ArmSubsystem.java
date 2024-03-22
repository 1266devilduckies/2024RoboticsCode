package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
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
        NEUTRAL(30),
        SHOOT_SPEAKER_FRONT(100),
        GROUND_PICKUP(110), 
        SHOOT_SPEAKER_SIDE_CORNER(100);

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

    PIDController pidController = new PIDController(Constants.Arm.armP, 0, Constants.Arm.armD);

    CANcoder armEncoder = new CANcoder(Constants.Arm.encoderID);

    TalonFX armMotor = new TalonFX(Constants.Arm.armID);
    //TalonFX armMotorFollower = new TalonFX(Constants.Arm.armFollowerID);

    TalonFXConfiguration armMotorConfiguration;

    ArmState currentArmState = null;

    public ArmSubsystem(RobotContainer robotContainer){
        armMotorConfiguration = new TalonFXConfiguration();
        
        configure();
    }

    private void configure(){
        armMotorConfiguration.Slot0.kP = Constants.Arm.armDefaultP;
        armMotorConfiguration.Slot0.kI = Constants.Arm.armDefaultI;
        armMotorConfiguration.Slot0.kD = Constants.Arm.armDefaultD;

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        armMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        armMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30;
    }

    @Override
    public void periodic(){
        //DEBUG
        if(RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.elbowMovementAxis) > 0.05){
            armMotor.setVoltage(1);
        }
        else if(RobotContainer.operatorJoystick.getRawAxis(Constants.OperatorConstants.elbowMovementAxis) < -0.05){
            armMotor.setVoltage(-1);
        }
        else{
            armMotor.setVoltage(0);
        }
        //DEBUG

        SmartDashboard.putNumber("Arm Angle", getArmAngle());

        if(currentArmState == null) return;
        setArmAngle(currentArmState.getAngle());
    }

    public void setArmState(ArmState state){
        this.currentArmState = state;
    }

    public double getArmAngle(){
        // 1 degree = 11.37
        return armEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    private void setArmAngle(double angle){
        //double feedForward = armFeedforward.calculate(getArmAngle(), angle);
        double clampedVoltage = MathUtil.clamp(-pidController.calculate(getArmAngle(), angle), -4, 4);
        armMotor.setVoltage(clampedVoltage);
    }

    public boolean atSetpoint(){
        if(Math.abs(getArmAngle() - currentArmState.getAngle()) < 5){
            return true;
        }
        return false;
    }

}