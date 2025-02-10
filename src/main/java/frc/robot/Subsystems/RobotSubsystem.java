package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClampSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem.ArmState;
import frc.robot.Subsystems.Arm.WristSubsystem.WristState;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.DriveState;
import frc.robot.Subsystems.LEDs.LEDSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.ControlBoard.RobotConfigProvider;
import frc.robot.Utils.ControlBoard.RobotConfiguration;

public class RobotSubsystem extends SubsystemBase{
    ArmSubsystem arm;
    ClampSubsystem clamp;
    SwerveSubsystem drivebase;
    EndEffectorSubsystem endEffector;
    WristSubsystem wrist;
    LEDSubsystem leds;

    private RobotState robotState = RobotState.I_IDLE;
    private RobotState endRobotState = RobotState.I_IDLE;
    private RobotConfiguration config;

    public RobotSubsystem(ArmSubsystem arm, ClampSubsystem clamp, SwerveSubsystem drivebase, EndEffectorSubsystem endEffector, WristSubsystem wrist, LEDSubsystem leds) {
        this.arm = arm;
        this.clamp = clamp;
        this.drivebase = drivebase;
        this.endEffector = endEffector;
        this.wrist = wrist;
        this.leds = leds;

    }

    @Override
    public void periodic() {
        switch (robotState) {
            case T_TRAVELING:
                arm.setTargetAngle(config.travelArmAngle);
                wrist.setTargetAngle(config.travelWristAngle);

                double armAngle = arm.getArmAngle();
                if (armAngle >= ArmConstants.ARM_SAFE_MIN_ANGLE && armAngle <= ArmConstants.ARM_SAFE_MAX_ANGLE) {
                    drivebase.pathfindToOTFPath(config.desiredEndPose).schedule();
                    robotState = RobotState.P_PATHING;
                }
                break;
        
            case P_PATHING:
                if (drivebase.getDriveState() == DriveState.FOLLOWING_PATH) {
                    arm.setTargetAngle(config.finalArmAngle);
                    wrist.setTargetAngle(config.finalWristAngle);
                    robotState = endRobotState;
                }
                break;

            case F_FEEDER:
            case S_SCORING:
                if (drivebase.getDriveState() == DriveState.USER_CONTROLLED &&
                    arm.getState() == ArmState.STATIONARY &&
                    wrist.getWristState() == WristState.STATIONARY) {
                        endEffector.setState(config.endEffectorState);
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
        config = RobotConfigProvider.getFeederConfiguration(drivebase.getHeading().getDegrees());
        endRobotState = RobotState.F_FEEDER;
        robotState = RobotState.T_TRAVELING;
    }

    public void goToScore() {
        config = RobotConfigProvider.getGamePieceConfiguration(drivebase.getHeading().getDegrees(), endEffector.getGamePieceOffset());
        endRobotState = RobotState.S_SCORING;
        robotState = RobotState.T_TRAVELING;
    }

    public void goToCage() {
        config = RobotConfigProvider.getCageConfiguration();
        endRobotState = RobotState.C_CLIMBING;
        robotState = RobotState.T_TRAVELING;
    }


    public RobotState getState() {
        return robotState;
    }

    public enum RobotState {
        I_IDLE,
        C_CLIMBING,
        B_BRAKINGCLIMB,
        T_TRAVELING,
        P_PATHING,
        F_FEEDER,
        S_SCORING,
        M_MANUAL,
        W_WIN
    }
}
