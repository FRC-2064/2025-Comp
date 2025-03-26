package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANdi;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.ctre.KArmSubsystem;
import frc.robot.Subsystems.Arm.ctre.KWristSubsystem;
import frc.robot.Subsystems.Arm.rev.ArmSubsystem;
import frc.robot.Subsystems.Arm.rev.WristSubsystem;
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
    KArmSubsystem arm;
    ClimbSubsystem climb;
    SwerveSubsystem drivebase;
    EndEffectorSubsystem endEffector;
    KWristSubsystem wrist;
    LEDSubsystem leds;

    private CANdi candi = new CANdi(56);

    private RobotState robotState = RobotState.I_IDLE;
    private RobotState endRobotState = RobotState.I_IDLE;
    public RobotConfiguration config;

    private Command currentPathCommand;

    public RobotSubsystem(KArmSubsystem arm, ClimbSubsystem clamp, SwerveSubsystem drivebase,
            EndEffectorSubsystem endEffector, KWristSubsystem wrist, LEDSubsystem leds) {
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

                double armAngle = arm.getArmAngle();
                if ((armAngle >= ArmConstants.ARM_SAFE_MIN_ANGLE && armAngle <= ArmConstants.ARM_SAFE_MAX_ANGLE)
                        || Robot.isSimulation()) {
                    currentPathCommand = drivebase.pathfindToOTFPath(config.desiredStartPose, config.desiredEndPose);
                    currentPathCommand.schedule();
                    robotState = RobotState.P_PATHING;
                }
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
                // endEffector.setState(config.endEffectorState);
                if (drivebase.getDriveState() == DriveState.USER_CONTROLLED &&
                        arm.getState() == ArmState.STATIONARY &&
                        wrist.getState() == WristState.STATIONARY) {
                    // endEffector.setState(config.endEffectorState);
                    robotState = RobotState.I_IDLE;
                }

            case C_CLIMBING:
                // do climb stuff {}
                // if robot has climbed then brake climb
                robotState = RobotState.B_BRAKINGCLIMB;
            case B_BRAKINGCLIMB:
                // if braking is complete
                robotState = RobotState.W_WIN;
            case W_WIN:
                // do win stuff {}
                robotState = RobotState.I_IDLE;
            case I_IDLE:
            default:
                break;
        }

    }

    public void goToFeeder() {
        config = RobotConfigProvider.getFeederConfiguration();
        if (config == null) {
            return;
        }
        endRobotState = RobotState.F_FEEDER;
        robotState = RobotState.T_TRAVELING;
    }

    public void goToScore() {
        config = RobotConfigProvider.getGameScoreConfiguration(drivebase.getHeading().getDegrees());
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
                .getGameScoreConfiguration(drivebase.getHeading().getDegrees());
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
                .getFeederConfiguration();
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
            endEffector.setState(EndEffectorState.OUTTAKING_CORAL);
            return;
        }
        endEffector.setState(config.startEndEffectorState);
    }

    public void runOuttakeConfig() {
        if (config == null) {
            endEffector.setState(EndEffectorState.OUTTAKING_CORAL);
            return;
        }
        endEffector.setState(config.finalEndEffectorState);
    }

        public void climbSequence(){
         new SequentialCommandGroup(
            new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_UP_ANGLE)),
            new WaitCommand(0.75),
            new InstantCommand(climb::toggleClamp),
            new WaitCommand(0.2),
            new InstantCommand(() -> arm.setTargetAngle(ArmConstants.ARM_CLIMB_DOWN_ANGLE))
        ).schedule();
        }
            


}
