// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.ctre.KArmSubsystem;
import frc.robot.Subsystems.Arm.ctre.KWristSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.Enums.EndEffectorState;

public class AlgaeIntakeCmd extends Command {
  private KArmSubsystem arm;
  private EndEffectorSubsystem endEffector;
  private KWristSubsystem wrist;
  private boolean isFinished = false;
  public AlgaeIntakeCmd(KArmSubsystem arm, KWristSubsystem wrist, EndEffectorSubsystem endEffector) {
    this.arm = arm;
    this.wrist = wrist;
    this.endEffector = endEffector;
  }

  @Override
  public void initialize() {
    arm.setTargetAngle(ArmConstants.ARM_ALGAE_CARRY_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_ALGAE_ANGLE);
    endEffector.setState(EndEffectorState.INTAKING_ALGAE);
  }

  @Override
  public void end(boolean interrupted) {
    endEffector.setState(EndEffectorState.STOPPED);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
