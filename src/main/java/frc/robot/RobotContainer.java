// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControlBoardConstants;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  //final CommandXboxController operatorXbox = new CommandXboxController(1);
  final Joystick driverJoystick = new Joystick(0);
  final Joystick driverTurnJoystick = new Joystick(1);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  final ClampSubsystem clamp = new ClampSubsystem();
  final LEDSubsystem leds = new LEDSubsystem();
  final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swerve"));


  final RobotSubsystem robot = new RobotSubsystem(arm, clamp, drivebase, endEffector, wrist, leds);

  // set controlboard

  Command cb_set = new InstantCommand(() -> {
    ControlBoardHelpers.setLevel(1);
    ControlBoardHelpers.setReefLocation(ControlBoardConstants.REEF_LOCATION_L);
  });

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

  /// Joystick Commands for manual practice
  Command frontTroughReef = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_FRONT_ANGLE);
    });

    Command frontL2Reef = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_L2_FRONT_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L2_FRONT_ANGLE);
    });

    Command frontFeeder = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_FRONT_INTAKE_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_FRONT_INTAKE_ANGLE);
    });

    

    Command backTroughReef = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_TROUGH_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_TROUGH_BACK_ANGLE);
    });

    Command backL2Reef = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_L2_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L2_BACK_ANGLE);
    });

    Command backL3Reef = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_L3_BACK_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_L3_BACK_ANGLE);
    });

    Command frontLowAlgaeRemoval = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_LOW_ALGAE_REMOVAL_ANGLE);
    });

    Command backHighAlgaeRemoval = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE);
    });

    
    Command backFeeder = new InstantCommand(
    () -> {
      arm.setTargetAngle(ArmConstants.ARM_BACK_INTAKE_ANGLE);
    wrist.setTargetAngle(WristConstants.WRIST_BACK_INTAKE_ANGLE);
    });

    Command groundIntake = new InstantCommand(
      () -> {
        arm.setTargetAngle(ArmConstants.ARM_GROUND_INTAKE);
        wrist.setTargetAngle(WristConstants.WRIST_GROUND_INTAKE);
      }
    );
  

  // DRIVE COMMANDS
  Command driveFieldOrientedDirectAngle = drivebase.driveDirectAngle(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY());

  Command driveFieldOrientedDirectAngleJoystick = drivebase.driveDirectAngle(
      () -> MathUtil.applyDeadband(-driverJoystick.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverJoystick.getX(), OperatorConstants.DEADBAND),
      () -> -driverTurnJoystick.getX(),
      () -> -driverTurnJoystick.getY());

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1).withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityJoystick = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverJoystick.getY() * -1,
      () -> driverJoystick.getX() * -1).withControllerRotationAxis(() -> -driverTurnJoystick.getX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      Command driveFieldOrientedAnglularVelocityJoystick = drivebase.driveFieldOriented(driveAngularVelocityJoystick);
      
  public RobotContainer() {
    // REGISTER AUTO COMMANDS
    // NamedCommands.registerCommand("Intake", new InstantCommand(wrist::intakeCoral));
    // NamedCommands.registerCommand("stopIntake", new InstantCommand(wrist::stopIntakeMotors));
    // NamedCommands.registerCommand("Outtake", new InstantCommand(wrist::outtakeCoral));
    // NamedCommands.registerCommand("armToFloor", homeArm);

    NamedCommands.registerCommand("ArmToTrough", troughFront);
    NamedCommands.registerCommand("FrontL2", frontL2Reef);
    NamedCommands.registerCommand("BackL3", backL3Reef);
    NamedCommands.registerCommand("OuttakeEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.OUTTAKING_CORAL)));
    NamedCommands.registerCommand("StopEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.STOPPED)));
    NamedCommands.registerCommand("IntakeEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.INTAKING_CORAL)));
    NamedCommands.registerCommand("FeederFront", frontFeeder);
    configureBindings();

  }

  private void configureBindings() {

    ControlBoardHelpers.setScoreLocation("TEST");

    // 'GO TO' BINDINGS
    // UNTESTED DO NOT USE
    // driverXbox.a().onTrue(new InstantCommand(robot::goToFeeder));
    // driverXbox.b().onTrue(new InstantCommand(robot::goToCage));
    // driverXbox.x().onTrue(new InstantCommand(robot::goToScore));
    // driverXbox.a().onTrue(new InstantCommand(() -> robot.setState(RobotState.P_PATHING)));
    // driverXbox.b().onTrue(new InstantCommand(() -> robot.setState(RobotState.S_SCORING)));

    driverXbox.y().onTrue(cb_set);
    //driverXbox.x().onTrue(new InstantCommand(robot::goToScore));


    // ARM BINDINGS
    // driverXbox.y().onTrue(level3Back);
    //driverXbox.x().onTrue(intakeBack);
    // driverXbox.leftTrigger().onTrue(troughFront);
    // driverXbox.leftTrigger().whileTrue(outtakeCoral);
    // driverXbox.leftBumper().onTrue(level2Front);
    // driverXbox.rightTrigger().whileTrue(intakeCoral);

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
    // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityJoystick);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleJoystick);
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));
    // driverXbox.leftBumper()
    //     .whileTrue(drivebase.driveToPose(new Pose2d(11.6, 4, new Rotation2d(Units.degreesToRadians(1)))));
    // driverXbox.leftBumper().whileTrue(drivebase.lineUpWithTag(frontTX));



  // Intake outake
  // new JoystickButton(driverJoystick, 1).whileTrue(outtakeCoral);
  // new JoystickButton(driverTurnJoystick, 1).whileTrue(intakeCoral);

  driverXbox.rightTrigger().whileTrue(intakeCoral);
  driverXbox.leftTrigger().whileTrue(outtakeCoral);

  // Algage
  // new JoystickButton(driverTurnJoystick, 14).onTrue(intakeAlgaeCommand);
  // new JoystickButton(driverTurnJoystick, 15).onTrue(outtakeAlgaeCommand);
  
  

  // new JoystickButton(driverJoystick, 14).onTrue(frontTroughReef);
  // new JoystickButton(driverJoystick, 15).onTrue(frontL2Reef);
  // new JoystickButton(driverJoystick, 13).onTrue(frontLowAlgaeRemoval);

  operatorXbox.rightBumper().onTrue(frontL2Reef);
  operatorXbox.a().onTrue(frontTroughReef);
  operatorXbox.povRight().onTrue(frontLowAlgaeRemoval);
  operatorXbox.povLeft().onTrue(groundIntake);

  operatorXbox.povUp().onTrue(intakeAlgaeCommand);



  // new JoystickButton(driverJoystick, 8).onTrue(backTroughReef);
  // new JoystickButton(driverJoystick, 9).onTrue(backL2Reef);
  // new JoystickButton(driverJoystick, 10).onTrue(backL3Reef);
  // new JoystickButton(driverJoystick, 7).onTrue(backHighAlgaeRemoval);

  operatorXbox.x().onTrue(backTroughReef);
  operatorXbox.leftBumper().onTrue(backL2Reef);
  operatorXbox.y().onTrue(backL3Reef);
  driverXbox.b().onTrue(backHighAlgaeRemoval);

  // new JoystickButton(driverJoystick, 12).onTrue(frontFeeder);
  // new JoystickButton(driverJoystick, 6).onTrue(backFeeder);

  operatorXbox.rightTrigger().onTrue(frontFeeder);
  operatorXbox.leftTrigger().onTrue(backFeeder);

  



  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Big Brain 3");
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
