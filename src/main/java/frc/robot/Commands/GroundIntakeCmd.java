// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RobotSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.WristConstants;

public class GroundIntakeCmd extends Command {
  private ArmSubsystem arm;
  private EndEffectorSubsystem endEffector;
  private WristSubsystem wrist;
  private RobotSubsystem robot;
  
  public GroundIntakeCmd(ArmSubsystem arm, WristSubsystem wrist, EndEffectorSubsystem endEffector, RobotSubsystem robot) {
    this.arm = arm;
    this.wrist = wrist;
    this.endEffector = endEffector;
    this.robot = robot;
  }

  @Override
  public void initialize() {
    arm.setTargetAngle(ArmConstants.ARM_GROUND_INTAKE);
    wrist.setTargetAngle(WristConstants.WRIST_GROUND_INTAKE);
    endEffector.setState(EndEffectorState.INTAKING_CORAL);;
    if(robot.config != null){
      robot.config.setFinalEndEffectorState(EndEffectorState.OUTTAKING_CORAL);      
    }
  }

  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {
    endEffector.setState(EndEffectorState.STOPPED);
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_FRONT_ANGLE);
  }

  @Override
  public boolean isFinished() {
    return endEffector.hasCoral;
  }
}
