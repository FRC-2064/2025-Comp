// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShovelConstants;
import frc.robot.Subsystems.SwerveDrive.ShovelSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final ShovelSubsystem shovel = new ShovelSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(
      Filesystem.getDeployDirectory(), 
      "swerve"
    )
  );


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
    configureBindings();
  }

  private void configureBindings() {
        
      // SHOVEL BINDINGS 
      driverXbox.a().onTrue(new InstantCommand(() -> shovel.setShovelAngle(ShovelConstants.DUMP_ANGLE)));
      driverXbox.x().onTrue(new InstantCommand(() -> shovel.setShovelAngle(ShovelConstants.INTAKE_ANGLE)));

      // VISION BINDINGS
      driverXbox.y().onTrue(new InstantCommand(() -> drivebase.lineUpWithTag(LimelightHelpers.getTX("limelight"))));
      
      // DRIVE BINDINGS
      driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));  
      driverXbox.start().onTrue(new InstantCommand(drivebase::setPredefinedOdom));
      driverXbox.b().onTrue(drivebase.driveToPose(new Pose2d(2,2,new Rotation2d())));

      //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
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
