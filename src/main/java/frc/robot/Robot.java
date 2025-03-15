// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants.Limelight1Constants;
import frc.robot.Utils.Constants.Limelight2Constants;
import frc.robot.Utils.ControlBoard.AutoPoster;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public Robot() {
    m_robotContainer = new RobotContainer();
    PathfindingCommand.warmupCommand().schedule();
    AutoPoster.postAutos();
    // CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    LimelightHelpers.SetThrottle(Limelight1Constants.LIMELIGHT_NAME, 200);
    LimelightHelpers.SetThrottle(Limelight2Constants.LIMELIGHT_NAME, 200);
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {
    LimelightHelpers.SetThrottle(Limelight1Constants.LIMELIGHT_NAME, 0);
    LimelightHelpers.SetThrottle(Limelight2Constants.LIMELIGHT_NAME, 0);

  }

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.drivebase.getAutonomousCommand(ControlBoardHelpers.getAuto());
    // m_autonomousCommand = m_robotContainer.drivebase.getAutonomousCommand("J1 K1 L1 L1 K1");

    // m_robotContainer.drivebase.zeroGyro();

    m_autonomousCommand = m_robotContainer.getAuto();

    
    if (m_autonomousCommand != null) {
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
  public void teleopPeriodic() {
  }

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
