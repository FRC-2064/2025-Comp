// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Utils.Enums.EndEffectorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  EndEffectorSubsystem endEffector;
  public AutoIntake(EndEffectorSubsystem endEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setState(EndEffectorState.INTAKING_CORAL);    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setState(EndEffectorState.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.hasCoral;
  }
}
