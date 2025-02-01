// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.WristSubsystem;
import frc.robot.Subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final ArmSubsystem arm = new ArmSubsystem();
  final WristSubsystem wrist = new WristSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(
      Filesystem.getDeployDirectory(), 
      "swerve"
    )
  );

  DoubleSupplier frontTX = new DoubleSupplier() {
    public double getAsDouble() {
      return LimelightHelpers.getTX("limelight-one");
    };
  };

  // ARM COMMANDS
  Command homeArm = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.HOME_ANGLE));
  Command lowAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.LOW_ALGAE_REMOVAL_ANGLE));
  Command trough = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.TROUGH_ANGLE));
  Command highAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.HIGH_ALGAE_REMOVAL_ANGLE));
  Command carryAlgae = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ALGEA_CARRY_ANGLE));
  Command intakeFeederAngle = new InstantCommand(() -> arm.setTargetAngle(ArmConstants.FEEDER_ANGLE));


  Command toggleArmBrake = new InstantCommand(() -> arm.armToggleCoast());
  

  //CLIMB CLAMP COMMANDS
  Command toggleClamp = new InstantCommand(() -> arm.toggleClamp());

  // VISION COMMANDS
  //Command lineupWithTag = new InstantCommand(drivebase::lineUpWithTag);

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
    NamedCommands.registerCommand("Intake", new InstantCommand(wrist::intakeCoral));
    NamedCommands.registerCommand("stopIntake", new InstantCommand(wrist::stopIntakeMotors));
    NamedCommands.registerCommand("Outtake", new InstantCommand(wrist::outtakeCoral));
    NamedCommands.registerCommand("armToFloor", homeArm);
    NamedCommands.registerCommand("armToTrough", trough);

    // CAMERA
    

    configureBindings();
    
  }

  private void configureBindings() {
    


    
    // DRIVE BINDINGS
    //driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));  
    // driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));
    driverXbox.leftBumper().whileTrue(drivebase.driveToPose(new Pose2d(11.6,4,new Rotation2d(Units.degreesToRadians(1)))));
    driverXbox.b().onTrue(trough);
    driverXbox.y().onTrue(homeArm);
    driverXbox.leftBumper().onTrue(toggleClamp);
    // driverXbox.leftBumper().whileTrue(drivebase.lineUpWithTag(frontTX));
    driverXbox.rightBumper().onTrue(intakeFeederAngle);
    //driverXbox.start().onTrue(carryAlgae);
    driverXbox.start().onTrue(toggleArmBrake);
     

    driverXbox.a().onTrue(new InstantCommand(wrist::intakeCoral));
    // driverXbox.a().onFalse(new InstantCommand(arm::stopIntakeMotors));
    // driverXbox.x().onTrue(new InstantCommand(arm::outtakeCoral));
    driverXbox.x().onTrue(new InstantCommand(wrist::stopIntakeMotors));
  

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("1 Coral");
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
