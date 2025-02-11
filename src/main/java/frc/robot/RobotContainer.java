// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.RobotSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClampSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Subsystems.RobotSubsystem.RobotState;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.Utils.Constants.WristConstants;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  //final CommandXboxController operatorXbox = new CommandXboxController(1);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  final ClampSubsystem clamp = new ClampSubsystem();
  final LEDSubsystem leds = new LEDSubsystem();
  final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swervewrist"));


  final RobotSubsystem robot = new RobotSubsystem(arm, clamp, drivebase, endEffector, wrist, leds);

  // ARM COMMANDS
  Command homeArm = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_HOME_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_HOME_ANGLE);
  });
  Command troughFront = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_FRONT_ANGLE);
  });
  Command troughBack = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_BACK_ANGLE);
  });
  Command level2Front = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_L2_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L2_FRONT_ANGLE);
  });
  Command level2Back = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_L2_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L2_BACK_ANGLE);
  });

  Command level3Back = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.ARM_L3_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L3_BACK_ANGLE);
  });

  Command intakeBack = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.ARM_BACK_INTAKE_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_BACK_INTAKE_ANGLE);
  });

  Command intakeAlgaeCommand = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.ARM_ALGAE_CARRY_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_ALGAE_INTAKE_ANGLE);
  });

  Command removeAlgaeHigh = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE);
  });


  Command highAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE));
  Command lowAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE));
  
  Command toggleArmBrake = new InstantCommand(() -> 
  {
    arm.armToggleCoast();
    wrist.wristToggleCoast();
  });

  // EE COMMANDS
  Command intakeCoral = new StartEndCommand(
    () -> endEffector.setState(EndEffectorState.INTAKING_CORAL),
    () -> endEffector.setState(EndEffectorState.STOPPED),
    endEffector);

  Command outtakeCoral = new StartEndCommand(
    () -> endEffector.setState(EndEffectorState.OUTTAKING_CORAL),
    () -> endEffector.setState(EndEffectorState.STOPPED),
    endEffector);

  Command intakeAlgae = new StartEndCommand(
    () -> endEffector.setState(EndEffectorState.INTAKING_ALGAE),
    () -> endEffector.setState(EndEffectorState.STOPPED),
    endEffector);

  Command outtakeAlgae = new StartEndCommand(
    () -> endEffector.setState(EndEffectorState.OUTTAKING_ALGAE),
    () -> endEffector.setState(EndEffectorState.STOPPED),
    endEffector);

  Command removeHigh = new StartEndCommand(
      () -> endEffector.setState(EndEffectorState.REMOVING_HIGH_ALGAE),
      () -> endEffector.setState(EndEffectorState.STOPPED),
      endEffector);

  Command removeLow = new StartEndCommand(
      () -> endEffector.setState(EndEffectorState.REMOVING_LOW_ALGAE),
      () -> endEffector.setState(EndEffectorState.STOPPED),
      endEffector);

  // DRIVE COMMANDS
  Command driveFieldOrientedDirectAngle = drivebase.driveDirectAngle(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1).withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  public RobotContainer() {
    // REGISTER AUTO COMMANDS
    // NamedCommands.registerCommand("Intake", new InstantCommand(wrist::intakeCoral));
    // NamedCommands.registerCommand("stopIntake", new InstantCommand(wrist::stopIntakeMotors));
    // NamedCommands.registerCommand("Outtake", new InstantCommand(wrist::outtakeCoral));
    NamedCommands.registerCommand("armToFloor", homeArm);

    configureBindings();

  }

  private void configureBindings() {

    // 'GO TO' BINDINGS
    // UNTESTED DO NOT USE
    // driverXbox.a().onTrue(new InstantCommand(robot::goToFeeder));
    // driverXbox.b().onTrue(new InstantCommand(robot::goToCage));
    driverXbox.x().onTrue(new InstantCommand(robot::goToScore));
    driverXbox.a().onTrue(new InstantCommand(() -> robot.setState(RobotState.P_PATHING)));
    driverXbox.b().onTrue(new InstantCommand(() -> robot.setState(RobotState.S_SCORING)));


    // ARM BINDINGS
    // driverXbox.y().onTrue(level3Back);
    //driverXbox.x().onTrue(intakeBack);
    // driverXbox.leftTrigger().onTrue(troughFront);
    driverXbox.rightTrigger().whileTrue(outtakeCoral);
    // driverXbox.leftBumper().onTrue(level2Front);
    driverXbox.rightBumper().whileTrue(intakeCoral);

    driverXbox.start().onTrue(toggleArmBrake);

    // WRIST BINDINGS
    //driverXbox.a().whileTrue(intake);
    // driverXbox.x().whileTrue(outtake);
    // driverXbox.a().whileTrue(intakeAlgae);
    // driverXbox.x().whileTrue(outtakeAlgae);
    // driverXbox.leftTrigger().whileTrue(intakeAlgae);
    // driverXbox.leftBumper().whileTrue(outtakeAlgae);
    //driverXbox.b().onTrue(intakeAlgaeCommand);
    // driverXbox.y().onTrue(removeAlgaeHigh);

    // DRIVE BINDINGS
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));
    // driverXbox.leftBumper()
    //     .whileTrue(drivebase.driveToPose(new Pose2d(11.6, 4, new Rotation2d(Units.degreesToRadians(1)))));
    // driverXbox.leftBumper().whileTrue(drivebase.lineUpWithTag(frontTX));

  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("1 Coral");
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
