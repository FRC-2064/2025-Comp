// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AlgaeIntakeCmd;
import frc.robot.Commands.BasicCmd;
import frc.robot.Commands.GroundIntakeCmd;
import frc.robot.Commands.BasicCmd.ArmCommands;
import frc.robot.Commands.BasicCmd.ClimbCommands;
import frc.robot.Commands.BasicCmd.EndEffectorCommands;
import frc.robot.Subsystems.RobotSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // SUBSYSTEM INITS
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  final ClimbSubsystem climb = new ClimbSubsystem();
  final LEDSubsystem leds = new LEDSubsystem();
  final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swerve"));

  final RobotSubsystem robot = new RobotSubsystem(arm, climb, drivebase, endEffector, wrist, leds);

  // CUSTOM COMMANDS
  final GroundIntakeCmd groundIntake = new GroundIntakeCmd(arm, wrist, endEffector);
  final AlgaeIntakeCmd algaeIntake = new AlgaeIntakeCmd(arm, wrist, endEffector);

  // BASIC COMMANDS
  final BasicCmd base = new BasicCmd(wrist, arm, endEffector, climb);
  final ArmCommands armCmd = base.armCmd;
  final EndEffectorCommands eeCmd = base.eeCmd;
  final ClimbCommands climbCmd = base.climbCmd;


  // DRIVE COMMANDS
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1 * (driverXbox.getHID().getBButton() ? 0.25 : 1),
      () -> driverXbox.getLeftX() * -1 * (driverXbox.getHID().getBButton() ? 0.25 : 1))
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * (driverXbox.getHID().getBButton() ? 0.25 : 1))
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  public RobotContainer() {
    // ARM LOCATIONS
    NamedCommands.registerCommand("FrontL1", armCmd.FrontL1);
    NamedCommands.registerCommand("FrontL2", armCmd.FrontL2);
    NamedCommands.registerCommand("BackL2", armCmd.BackL2);
    NamedCommands.registerCommand("BackL3", armCmd.BackL3);
    NamedCommands.registerCommand("FrontFeeder", armCmd.FrontFeeder);
    NamedCommands.registerCommand("HighAlgae", armCmd.HighAlgae);
    NamedCommands.registerCommand("LowAlgae", armCmd.LowAlgae);
    NamedCommands.registerCommand("GroundIntake", groundIntake);

    // INTAKE STATES
    NamedCommands.registerCommand("OuttakeEE", eeCmd.OuttakeEE);
    NamedCommands.registerCommand("IntakeEE", eeCmd.IntakeEE);
    NamedCommands.registerCommand("HighEE", eeCmd.HighEE);
    NamedCommands.registerCommand("LowEE", eeCmd.LowEE);
    NamedCommands.registerCommand("StopEE", eeCmd.StopEE);
    configureBindings();

  }

  private void configureBindings() {

    // GAME PIECE MANIPULATION
    driverXbox.leftTrigger().whileTrue(eeCmd.IntakeEE);
    driverXbox.rightTrigger().whileTrue(eeCmd.OuttakeEE);
    driverXbox.leftBumper().whileTrue(groundIntake);
    driverXbox.rightBumper().whileTrue(algaeIntake);

    driverXbox.a().onTrue(new InstantCommand(robot::goToScore));
    driverXbox.y().onTrue(new InstantCommand(robot::setArm));
    driverXbox.x().onTrue(armCmd.FrontFeeder);

    // CLIMB BINDINGS
    driverXbox.povDown().onTrue(armCmd.climbDown);
    driverXbox.povUp().onTrue(armCmd.climbUp);
    driverXbox.povRight().onTrue(climbCmd.winchIn);
    driverXbox.povLeft().onTrue(climbCmd.toggleClamp);

    // UTILITY BINDINGS
    driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    driverXbox.back().onTrue(armCmd.toggleArmBrake);

    // DEFAULT DRIVE
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // CANCELS OTF WHEN DRIVER MOVES
    new Trigger(() -> Math.abs(driverXbox.getLeftY()) > OperatorConstants.DEADBAND ||
                  Math.abs(driverXbox.getLeftX()) > OperatorConstants.DEADBAND ||
                  Math.abs(driverXbox.getRightX()) > OperatorConstants.DEADBAND)
    .onTrue(new InstantCommand(() -> drivebase.setState(DriveState.USER_CONTROLLED), drivebase));
  }

}
