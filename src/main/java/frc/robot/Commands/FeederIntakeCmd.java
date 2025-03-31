// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Utils.Enums.EndEffectorState;

public class FeederIntakeCmd extends Command {
  EndEffectorSubsystem endEffector;
  public FeederIntakeCmd(EndEffectorSubsystem endEffector) {
    this.endEffector = endEffector;
  }

  @Override
  public void initialize() {
    endEffector.setState(EndEffectorState.INTAKING_CORAL_FEEDER);    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    endEffector.setState(EndEffectorState.STOPPED);
  }

  @Override
  public boolean isFinished() {
    return endEffector.hasCoral;
  }
}
