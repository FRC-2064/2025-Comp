// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import com.ctre.phoenix6.hardware.CANdi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.ctre.KArmSubsystem;
import frc.robot.Subsystems.Arm.ctre.KWristSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // SUBSYSTEM INITS
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CANdi candi = new CANdi(Constants.CANDI_ID);
  final KArmSubsystem arm = new KArmSubsystem(candi);
  final KWristSubsystem wrist = new KWristSubsystem(candi);
  final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  final ClimbSubsystem climb = new ClimbSubsystem();
  final LEDSubsystem leds = new LEDSubsystem();
  final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(
          Filesystem.getDeployDirectory(),
          "swerve"));
          
          final RobotSubsystem robot = new RobotSubsystem(arm, climb, drivebase, endEffector, wrist, leds);
          
          // CUSTOM COMMANDS
          final GroundIntakeCmd groundIntake = new GroundIntakeCmd(arm, wrist, endEffector, robot);
          final AlgaeIntakeCmd algaeIntake = new AlgaeIntakeCmd(arm, wrist, endEffector);
          
          // BASIC COMMANDS
          final BasicCmd base = new BasicCmd(wrist, arm, endEffector, climb, robot);
          final ArmCommands armCmd = base.armCmd;
          final EndEffectorCommands eeCmd = base.eeCmd;
          final ClimbCommands climbCmd = base.climbCmd;
          
          private final SendableChooser<Command> autoChooser;
          

  // DRIVE COMMANDS
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1 * (driverXbox.getHID().getRightBumperButton() ? 0.25 : 0.75),
      () -> driverXbox.getLeftX() * -1 * (driverXbox.getHID().getRightBumperButton() ? 0.25 : 0.75))
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * (driverXbox.getHID().getRightBumperButton() ? 0.25 : 0.75))
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveRobotOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity.copy().allianceRelativeControl(false).robotRelative(true));


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
    NamedCommands.registerCommand("OuttakeEE", eeCmd.PPOuttakeEE);
    NamedCommands.registerCommand("IntakeEE", eeCmd.PPIntakeEE);
    NamedCommands.registerCommand("HighEE", eeCmd.PPHighEE);
    NamedCommands.registerCommand("LowEE", eeCmd.PPLowEE);
    NamedCommands.registerCommand("StopEE", eeCmd.PPStopEE);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

  }

  private void configureBindings() {

    // driverXbox.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // driverXbox.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    
    // driverXbox.y().whileTrue(arm.sysIdQuasistatic(Direction.kForward));
    // driverXbox.a().whileTrue(arm.sysIdQuasistatic(Direction.kReverse));
    // driverXbox.b().whileTrue(arm.sysIdDynamic(Direction.kForward));
    // driverXbox.x().whileTrue(arm.sysIdDynamic(Direction.kReverse));
    
    

    // GAME PIECE MANIPULATION
    driverXbox.leftTrigger().whileTrue(eeCmd.ControlBoardEEOuttake);
    driverXbox.rightTrigger().whileTrue(eeCmd.ControlBoardEEIntake);
    driverXbox.leftBumper().whileTrue(groundIntake);
    
    driverXbox.b().onTrue(new InstantCommand(robot::goToFeeder));
    driverXbox.a().onTrue(new InstantCommand(robot::goToScore));
    driverXbox.y().onTrue(new InstantCommand(robot::armToScore));
    driverXbox.x().onTrue(armCmd.FrontFeeder);

    // CLIMB BINDINGS
    driverXbox.povDown().onTrue(armCmd.climbDown);
    driverXbox.povUp().onTrue(armCmd.climbUp);
    driverXbox.povRight().whileTrue(climbCmd.winchIn);
    driverXbox.povLeft().onTrue(climbCmd.toggleClamp);

    // UTILITY BINDINGS
    driverXbox.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    driverXbox.back().onTrue(armCmd.toggleArmBrake);

    // DEFAULT DRIVE
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    new Trigger(() -> driverXbox.getHID().getRightBumperButton()).whileTrue(driveRobotOrientedAnglularVelocity);

    // CANCELS OTF WHEN DRIVER MOVES
    new Trigger(() -> Math.abs(driverXbox.getLeftY()) > OperatorConstants.DEADBAND ||
                  Math.abs(driverXbox.getLeftX()) > OperatorConstants.DEADBAND ||
                  Math.abs(driverXbox.getRightX()) > OperatorConstants.DEADBAND)
    .onTrue(new InstantCommand(() -> drivebase.setState(DriveState.USER_CONTROLLED), drivebase));
   }

  public Command getAuto() {
    return autoChooser.getSelected();
  }

}
