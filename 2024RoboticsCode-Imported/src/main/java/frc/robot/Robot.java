// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private PowerDistribution powerBoard;

  private DoubleEntry waitDelayEntry;
  private double waitDelay = 0.0;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    powerBoard = new PowerDistribution(1, ModuleType.kRev);
    powerBoard.clearStickyFaults();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleTopic waitDelayTopic = new DoubleTopic(nt.getTopic("/SmartDashboard/Wait Delay (Seconds)"));
    waitDelayEntry = waitDelayTopic.getEntry(0.0);
    waitDelayEntry.setDefault(waitDelay);

    Autos.pushAutosToDashboard(m_robotContainer.autonomousMode, m_robotContainer.drivetrainSubsystem);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (waitDelayEntry.get() != waitDelay) {
      waitDelay = waitDelayEntry.get();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutoCommandGroup();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      //Add -> reset odometry
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
