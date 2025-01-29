// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShovelConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ShovelSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final ShovelSubsystem shovel = new ShovelSubsystem();
  final ArmSubsystem arm = new ArmSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(
      Filesystem.getDeployDirectory(), 
      "swerve"
    )
  );

  // SHOVEL COMMANDS
  Command dump = new InstantCommand(() -> shovel.setShovelAngle(ShovelConstants.DUMP_ANGLE));
  Command intake = new InstantCommand(() -> shovel.setShovelAngle(ShovelConstants.INTAKE_ANGLE));

  // ARM COMMANDS
  Command homeArm = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.HOME_ANGLE));
  Command lowAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.LOW_ALGAE_REMOVAL_ANGLE));
  

  //CLIMB CLAMP COMMANDS
  Command hookClamp = new InstantCommand(() -> arm.setClimbClampAngle(ArmConstants.CLIMB_CLAMP_ANGLE));
  Command unhookClamp = new InstantCommand(() -> arm.setClimbClampAngle(ArmConstants.HOME_CLAMP_ANGLE));

  // VISION COMMANDS
  Command lineupWithTag = new InstantCommand(drivebase::lineUpWithTag);

  // DRIVE COMMANDS
  Command driveFieldOrientedDirectAngle = drivebase.driveDirectAngle(
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.DEADBAND),
    () -> -driverXbox.getRightX(),
    () -> -driverXbox.getRightY());

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1
  ).withControllerRotationAxis(driverXbox::getRightX)
   .deadband(OperatorConstants.DEADBAND)
   .scaleTranslation(0.8)
   .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  public RobotContainer() {
    // REGISTER AUTO COMMANDS
    NamedCommands.registerCommand("Dump", dump);
    NamedCommands.registerCommand("Intake", intake);
    configureBindings();
  }

  private void configureBindings() {
    // SHOVEL BINDINGS 
    // driverXbox.a().onTrue(dump);
    // driverXbox.x().onTrue(intake);

    // VISION BINDINGS
    // driverXbox.y().onTrue(lineupWithTag);
    
    // DRIVE BINDINGS
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));  
    driverXbox.start().onTrue(new InstantCommand(drivebase::setPredefinedOdom));
    // driverXbox.b().onTrue(drivebase.driveToPose(new Pose2d(2,2,new Rotation2d())));
    driverXbox.b().onTrue(lowAlgae);
    driverXbox.y().onTrue(homeArm);
    driverXbox.rightTrigger().onTrue(hookClamp);
    driverXbox.leftBumper().onTrue(unhookClamp);

    driverXbox.a().onTrue(new InstantCommand(arm::intakeCoral));
    // driverXbox.a().onFalse(new InstantCommand(arm::stopIntakeMotors));
    // driverXbox.x().onTrue(new InstantCommand(arm::outtakeCoral));
    driverXbox.x().onTrue(new InstantCommand(arm::stopIntakeMotors));
  

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Straight");
  }
  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
