// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.WristConstants;

public class AlgaeIntakeCmd extends Command {
  private ArmSubsystem arm;
  private EndEffectorSubsystem endEffector;
  private WristSubsystem wrist;
  private boolean isFinished = false;
  public AlgaeIntakeCmd(ArmSubsystem arm, WristSubsystem wrist, EndEffectorSubsystem endEffector) {
    this.arm = arm;
    this.wrist = wrist;
    this.endEffector = endEffector;
  }

  @Override
  public void initialize() {
    arm.setTargetAngle(ArmConstants.ARM_ALGAE_CARRY_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_ALGAE_ANGLE);
    endEffector.outtakeCoral();
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
