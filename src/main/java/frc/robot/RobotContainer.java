// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveDrive.AbsoluteFieldDrive;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.ShovelSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(
      Filesystem.getDeployDirectory(), 
      "swerve"
    )
  );


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommandTest(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());
        
  AbsoluteFieldDrive absfield = new AbsoluteFieldDrive(
    drivebase,
    () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND), 
    () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND)
    );
  // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(
  //   drivebase,
  //   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  //   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND), 
  //   () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
  //   driverXbox.getHID()::getYButtonPressed,
  //   driverXbox.getHID()::getAButtonPressed,
  //   driverXbox.getHID()::getXButtonPressed,
  //   driverXbox.getHID()::getBButtonPressed
  // );


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1
  ).withControllerRotationAxis(driverXbox::getRightX)
   .deadband(OperatorConstants.DEADBAND)
   .scaleTranslation(0.8)
   .allianceRelativeControl(true);

  ShovelSubsystem sm = new ShovelSubsystem();


  //  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
  //     driverXbox::getRightX,
  //     driverXbox::getRightY
  //   ).headingWhile(true);

  //   Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  
  

    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(
    //   drivebase.getSwerveDrive(),
    //   () -> -driverXbox.getLeftY(),
    //   () -> -driverXbox.getLeftX()
    //   ).withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
    //    .deadband(OperatorConstants.DEADBAND)
    //    .scaleTranslation(0.8)
    //    .allianceRelativeControl(true);

    // SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy().withControllerHeadingAxis(
    //   () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
    //   () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)
    //   ).headingWhile(true);

    // Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    //drivebase.setDefaultCommand(absfield);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(drivebase.driveCommandold(
    //   () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND), 
    //   () -> -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND)
    // )
    // );
    
    // driverXbox.a().whileTrue(drivebase.lineUpWithTag(LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME)));
    // driverXbox.a().whileTrue(
    // drivebase.lineUpWithTag(() -> LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME)));  
    driverXbox.b().onTrue(new InstantCommand(drivebase::zeroGyro));  
    driverXbox.a().onTrue(new InstantCommand(sm::pickUp));
    driverXbox.x().onTrue(new InstantCommand(sm::carry));
    driverXbox.y().onTrue(new InstantCommand(sm::dump));     
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("New Auto");
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
