// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlgaeIntakeCmd;
import frc.robot.Commands.GroundIntakeCmd;
import frc.robot.Subsystems.RobotSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControlBoardConstants;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  //final CommandXboxController operatorXbox = new CommandXboxController(1);
  final Joystick driverJoystick = new Joystick(0);
  final Joystick driverTurnJoystick = new Joystick(1);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  final ClimbSubsystem clamp = new ClimbSubsystem();
  final LEDSubsystem leds = new LEDSubsystem();
  final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swerve"));

  final GroundIntakeCmd groundIntake = new GroundIntakeCmd(arm, wrist, endEffector);
  final AlgaeIntakeCmd algaeIntake = new AlgaeIntakeCmd(arm, wrist, endEffector);


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

  Command winchIn = new StartEndCommand(
    clamp::winchIn,
    clamp::winchStop,
    clamp);
  
  Command winchOut = new StartEndCommand(
    clamp::winchOut,
    clamp::winchStop,
    clamp);


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
    // ARM LOCATIONS
    NamedCommands.registerCommand("FrontL1", troughFront);
    NamedCommands.registerCommand("FrontL2", frontL2Reef);
    NamedCommands.registerCommand("BackL2", backL2Reef);
    NamedCommands.registerCommand("BackL3", backL3Reef);
    NamedCommands.registerCommand("FrontFeeder", frontFeeder);
    NamedCommands.registerCommand("BackFeeder", backFeeder);
    NamedCommands.registerCommand("HighAlgae", highAlgae);
    NamedCommands.registerCommand("LowAlgae", lowAlgae);
    
    // INTAKE STATES
    NamedCommands.registerCommand("OuttakeEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.OUTTAKING_CORAL)));
    NamedCommands.registerCommand("IntakeEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.INTAKING_CORAL)));
    NamedCommands.registerCommand("HighEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.REMOVING_HIGH_ALGAE)));
    NamedCommands.registerCommand("LowEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.REMOVING_LOW_ALGAE)));
    NamedCommands.registerCommand("StopEE", new InstantCommand(() -> endEffector.setState(EndEffectorState.STOPPED)));
    configureBindings();

  }

  private void configureBindings() {


  // END EFFECTOR BASED
  driverXbox.leftTrigger().whileTrue(intakeCoral);
  driverXbox.rightTrigger().whileTrue(outtakeCoral);
  driverXbox.leftBumper().whileTrue(groundIntake);
  driverXbox.rightBumper().whileTrue(algaeIntake);

  // CONTROLBOARD BASED

  driverXbox.a().onTrue(new InstantCommand(robot::goToScore));
  driverXbox.b().onTrue(new InstantCommand(robot::goToFeeder));
  driverXbox.y().onTrue(new InstantCommand(robot::setArm));
  driverXbox.x().onTrue(frontFeeder);
    
  // CLIMB BINDINGS
  driverXbox.povDown().onTrue(new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_DOWN_ANGLE)));
  driverXbox.povUp().onTrue(new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_UP_ANGLE)));
  driverXbox.povRight().onTrue(winchIn);
  driverXbox.povLeft().onTrue(new InstantCommand(clamp::toggleClamp));

  //bad stuff
  driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));
  driverXbox.back().onTrue(toggleArmBrake);


  // DEFAULT DRIVE
  drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }
  
  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
