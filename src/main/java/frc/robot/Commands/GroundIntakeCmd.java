// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.security.spec.ECPublicKeySpec;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeCmd extends Command {
  /** Creates a new GroundIntakeCmd. */
  private ArmSubsystem arm;
  private EndEffectorSubsystem endEffector;
  private WristSubsystem wrist;
  private boolean isFinished = false;
  
  public GroundIntakeCmd(ArmSubsystem arm, WristSubsystem wrist, EndEffectorSubsystem endEffector) {
    this.arm = arm;
    this.wrist = wrist;
    this.endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTargetAngle(ArmConstants.ARM_GROUND_INTAKE);
    wrist.setTargetAngle(WristConstants.WRIST_GROUND_INTAKE);
    endEffector.intakeCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_FRONT_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
