// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.IntakeSubsystem.IntakeSpinState;

public class RobotContainer {

  public DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(this);
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem(this);
  public ArmSubsystem armSubsystem = new ArmSubsystem(this);

  public SendableChooser<SequentialCommandGroup> autonomousMode = new SendableChooser<SequentialCommandGroup>();

  public final static CommandPS4Controller driverJoystick =
      new CommandPS4Controller(DriverConstants.port);
  public final static CommandXboxController operatorJoystick =
      new CommandXboxController(OperatorConstants.port);

  public RobotContainer() {
    Shuffleboard.getTab("autos").add(autonomousMode);
    drivetrainSubsystem.setupPathPlanner();

    registerNamedCommands();
    configureBindings();
  }

  private void registerNamedCommands(){
    NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
      new MoveArm(armSubsystem, ArmState.GROUND_PICKUP), 
      new IntakeSpin(intakeSubsystem, IntakeSpinState.TAKE_IN)));

    NamedCommands.registerCommand("StopIntake", new SequentialCommandGroup(
      new MoveArm(armSubsystem, ArmState.INITIAL),
      new IntakeSpin(intakeSubsystem, IntakeSpinState.STOPPED)));
  }

  private void configureBindings() {
    driverJoystick.cross().onTrue(new SequentialCommandGroup(
      new MoveArm(armSubsystem, ArmState.GROUND_PICKUP), 
      new IntakeSpin(intakeSubsystem, IntakeSpinState.TAKE_IN)));
    driverJoystick.cross().onFalse(new SequentialCommandGroup(
      new MoveArm(armSubsystem, ArmState.INITIAL),
      new IntakeSpin(intakeSubsystem, IntakeSpinState.STOPPED)));\
  }

  public SequentialCommandGroup getAutoCommandGroup(){
    return autonomousMode.getSelected();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public DrivetrainSubsystem getDrivetrainSubsystem(){
    return drivetrainSubsystem;
  }
  
}