// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swervewrist"));

  // ARM COMMANDS
  Command homeArm = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_HOME_ANGLE);
    wrist.setWristAngle(WristConstants.HOME_ANGLE);
  });
  Command troughFront = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
    wrist.setWristAngle(WristConstants.TROUGH_FRONT_ANGLE);
  });
  Command troughBack = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.ARM_TROUGH_BACK_ANGLE);
    wrist.setWristAngle(WristConstants.TROUGH_BACK_ANGLE);
  });
  Command level2Front = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.L2_FRONT_ANGLE);
    wrist.setWristAngle(WristConstants.L2_FRONT_ANGLE);
  });
  Command level2Back = new InstantCommand(() -> 
  {
    arm.setTargetAngle(ArmConstants.L2_BACK_ANGLE);
    wrist.setWristAngle(WristConstants.L2_BACK_ANGLE);
  });

  Command level3Back = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.L3_BACK_ANGLE);
    wrist.setWristAngle(WristConstants.L3_BACK_ANGLE);
  });

  Command intakeBack = new InstantCommand(() ->
  {
    arm.setTargetAngle(ArmConstants.ARM_BACK_INTAKE_ANGLE);
    wrist.setWristAngle(WristConstants.BACK_INTAKE_ANGLE);
  });


  Command highAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.HIGH_ALGAE_REMOVAL_ANGLE));
  Command lowAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.LOW_ALGAE_REMOVAL_ANGLE));
  
  Command toggleArmBrake = new InstantCommand(() -> 
  {
    arm.armToggleCoast();
    wrist.wristToggleCoast();
  });



  // CLIMB CLAMP COMMANDS
  //Command toggleClamp = new InstantCommand(() -> arm.toggleClamp());

  // VISION COMMANDS
  DoubleSupplier frontTX = new DoubleSupplier() {
    public double getAsDouble() {
      return LimelightHelpers.getTX("limelight-one");
    };
  };

  //WRIST COMMANDS
  Command intake = new StartEndCommand(
      () -> wrist.intakeCoral(),
      () -> wrist.stopIntakeMotors(),
      wrist);
  Command outtake = new StartEndCommand(
      () -> wrist.outtakeCoral(),
      () -> wrist.stopIntakeMotors(),
      wrist);
    

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
    operatorXbox.a().onTrue(drivebase.goToScore());
    operatorXbox.x().onTrue(drivebase.goToCage());
    operatorXbox.b().onTrue(drivebase.goToFeeder());

    // ARM BINDINGS
    driverXbox.y().onTrue(level3Back);
    driverXbox.x().onTrue(intakeBack);
    driverXbox.leftTrigger().onTrue(troughFront);
    driverXbox.rightTrigger().onTrue(troughBack);
    driverXbox.leftBumper().onTrue(level2Front);
    driverXbox.rightBumper().onTrue(level2Back);

    driverXbox.start().onTrue(toggleArmBrake);

    // WRIST BINDINGS
    driverXbox.a().whileTrue(intake);
    driverXbox.b().whileTrue(outtake);

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
