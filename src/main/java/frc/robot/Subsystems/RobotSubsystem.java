package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Enums.ArmState;
import frc.robot.Utils.Enums.EndEffectorState;
import frc.robot.Utils.Enums.RobotState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;
import frc.robot.Utils.ControlBoard.RobotConfigProvider;
import frc.robot.Utils.ControlBoard.RobotConfiguration;

public class RobotSubsystem extends SubsystemBase {
    ArmSubsystem arm;
    ClimbSubsystem climb;
    SwerveSubsystem drivebase;
    EndEffectorSubsystem endEffector;
    WristSubsystem wrist;
    LEDSubsystem leds;

    private RobotState robotState = RobotState.I_IDLE;
    private RobotState endRobotState = RobotState.I_IDLE;
    public RobotConfiguration config;

    private Command currentPathCommand;

    public RobotSubsystem(ArmSubsystem arm, ClimbSubsystem clamp, SwerveSubsystem drivebase,
            EndEffectorSubsystem endEffector, WristSubsystem wrist, LEDSubsystem leds) {
        this.arm = arm;
        this.climb = clamp;
        this.drivebase = drivebase;
        this.endEffector = endEffector;
        this.wrist = wrist;
        this.leds = leds;

    }

    @Override
    public void periodic() {
        ControlBoardHelpers.updateRobotStatus();
        switch (robotState) {
            case T_TRAVELING:
                arm.setTargetAngle(config.travelArmAngle);
                wrist.setTargetAngle(config.travelWristAngle);
                if (currentPathCommand == null || !currentPathCommand.isScheduled()) {
                    Command otfPath = drivebase.pathfindToOTFPath(config.pathPoses);
                    Command driverCancel = new WaitUntilCommand(() -> RobotContainer.isDriverInputDetected());
                    currentPathCommand = new ParallelRaceGroup(otfPath, driverCancel)
                            .andThen(new InstantCommand(() -> {
                                drivebase.setState(DriveState.USER_CONTROLLED);
                            }, drivebase));
                    currentPathCommand.schedule();
                }
                robotState = RobotState.P_PATHING;
                break;

            case P_PATHING:
                if (drivebase.getDriveState() == DriveState.FOLLOWING_PATH) {
                    arm.setTargetAngle(config.finalArmAngle);
                    wrist.setTargetAngle(config.finalWristAngle);
                    robotState = endRobotState;
                }

                if (drivebase.getDriveState() == DriveState.USER_CONTROLLED) {
                    currentPathCommand.cancel();
                    robotState = RobotState.I_IDLE;
                }
                
                break;

            case F_FEEDER:
            case S_SCORING:
                if (drivebase.getDriveState() == DriveState.USER_CONTROLLED &&
                        arm.getState() == ArmState.STATIONARY &&
                        wrist.getState() == WristState.STATIONARY) {
                    Command cmd = new SequentialCommandGroup(
                            new InstantCommand(
                                    () -> endEffector.setState(config.finalEndEffectorState)),
                            new WaitCommand(0.5),
                            new InstantCommand(
                                    () -> endEffector.setState(EndEffectorState.STOPPED)));
                    cmd.schedule();
                    robotState = RobotState.I_IDLE;
                }

            case C_CLIMBING:
                robotState = RobotState.B_BRAKINGCLIMB;
            case B_BRAKINGCLIMB:
                robotState = RobotState.W_WIN;
            case W_WIN:
                robotState = RobotState.I_IDLE;
            case I_IDLE:
            default:
                break;
        }

    }

    public void goToFeeder() {
        config = RobotConfigProvider.getFeederConfiguration(drivebase.getPose());
        if (config == null) {
            return;
        }
        endRobotState = RobotState.F_FEEDER;
        robotState = RobotState.T_TRAVELING;
    }

    public void goToScore() {
        config = RobotConfigProvider.getGameScoreConfiguration(drivebase.getPose(),
                drivebase.getHeading().getDegrees());
        if (config == null) {
            return;
        }
        SmartDashboard.putString("Logging/Robot/EEConfig", config.finalEndEffectorState.toString());
        endRobotState = RobotState.S_SCORING;
        robotState = RobotState.T_TRAVELING;

    }

    public void armToScore() {
        robotState = RobotState.I_IDLE;
        config = RobotConfigProvider
                .getGameScoreConfiguration(drivebase.getPose(), drivebase.getHeading().getDegrees());
        if (config == null) {
            return;
        }
        if (config.finalArmAngle == ArmConstants.ARM_CLIMB_DOWN_ANGLE
                && arm.getTargetArmAngle() == ArmConstants.ARM_CLIMB_DOWN_ANGLE) {
            climbSequence();
            return;
        }
        arm.setTargetAngle(config.finalArmAngle);
        wrist.setTargetAngle(config.finalWristAngle);
    }

    public void armToFeeder() {
        robotState = RobotState.I_IDLE;
        config = RobotConfigProvider
                .getFeederConfiguration(drivebase.getPose());
        if (config == null) {
            return;
        }
        arm.setTargetAngle(config.finalArmAngle);
        wrist.setTargetAngle(config.finalWristAngle);
    }

    public RobotState getState() {
        return robotState;
    }

    public void setState(RobotState state) {
        robotState = state;
    }

    public void runIntakeConfig() {
        if (config == null) {
            endEffector.setState(EndEffectorState.OUTTAKING_TROUGH);
            return;
        }
        endEffector.setState(config.startEndEffectorState);
    }

    public void runOuttakeConfig() {
        if (config == null) {
            endEffector.setState(EndEffectorState.OUTTAKING_TROUGH);
            return;
        }
        endEffector.setState(config.finalEndEffectorState);
    }

    public void climbSequence() {
        new SequentialCommandGroup(
                new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_UP_ANGLE)),
                new WaitCommand(0.75),
                new InstantCommand(climb::toggleClamp),
                new WaitCommand(0.2),
                new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_DOWN_ANGLE)),
                new InstantCommand(() -> robotState = RobotState.C_CLIMBING)).schedule();
    }

}
