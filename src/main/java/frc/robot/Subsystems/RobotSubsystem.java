package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlBoard.ScoreConfiguration;
import frc.robot.Subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.ControlBoard.ScoreConfigProvider;

public class RobotSubsystem extends SubsystemBase{
    ArmSubsystem arm;
    ClampSubsystem clamp;
    SwerveSubsystem drivebase;
    EndEffectorSubsystem endEffector;
    WristSubsystem wrist;

    private RobotState currentState = RobotState.IDLE;
    private RobotState desiredState = RobotState.IDLE;

    private ScoreConfiguration config;


    public RobotSubsystem(ArmSubsystem arm, ClampSubsystem clamp, SwerveSubsystem drivebase, EndEffectorSubsystem endEffector, WristSubsystem wrist) {
        this.arm = arm;
        this.clamp = clamp;
        this.drivebase = drivebase;
        this.endEffector = endEffector;
        this.wrist = wrist;
    }

    @Override
    public void periodic() {
        if (currentState != desiredState && config != null) {
            manageState();
        }
    }

    private void manageState() {
        switch (desiredState) {
            case IDLE:
                break;
            case T_TRAVELING:
                t();
                break;
            case S_STARTEDPATH:
                s();
                break;
            case F_FEEDER:
                f();
                break;
            case R1_REEFLEVEL1, R2_REEFLEVEL2, R3_REEFLEVEL3: 
                r();
                break;
            case A1_LOWALGAE:
                a1();
                break;
            case A2_HIGHALGAE:
                a2();
                break;
            default:
                break;
        }
        currentState = desiredState;
    }

    private void t() {
        drivebase.pathfindToOTFPath(config.desiredEndPose).schedule();
        arm.setTargetAngle(config.travelArmAngle);
        wrist.setTargetAngle(config.travelWristAngle);
    }

    private void s() {
        arm.setTargetAngle(config.finalArmAngle);
        wrist.setTargetAngle(config.finalWristAngle);
    }

    private void f() {
        endEffector.setState(EndEffectorState.INTAKING_CORAL);
    }

    private void r() {
        endEffector.setState(EndEffectorState.OUTTAKING_CORAL);
    }

    private void a1() {
        endEffector.setState(EndEffectorState.REMOVING_LOW_ALGAE);
    }

    private void a2() {
        endEffector.setState(EndEffectorState.REMOVING_HIGH_ALGAE);
    }

    public RobotState getState() {
        return currentState;
    }

    public void setState(RobotState state) {
        desiredState = state;
    }

    public void goToFeeder() { 
        config = ScoreConfigProvider.getFeederConfiguration(drivebase.getHeading().getDegrees());
        currentState = RobotState.IDLE;
        desiredState = RobotState.T_TRAVELING;
    }

    public void goToScore() { 
        config = ScoreConfigProvider.getGamePieceConfiguration(drivebase.getHeading().getDegrees());
        currentState = RobotState.IDLE;
        desiredState = RobotState.T_TRAVELING;
    }

    public void goToCage() { 
        config = ScoreConfigProvider.getCageConfiguration();
        currentState = RobotState.IDLE;
        desiredState = RobotState.T_TRAVELING;
    }

    public enum RobotState {
        IDLE,
        C_CLIMBING,
        B_BRAKEDCLIMB,
        P_PROCESSOR,
        T_TRAVELING,
        S_STARTEDPATH,
        F_FEEDER,
        R1_REEFLEVEL1,
        R2_REEFLEVEL2,
        R3_REEFLEVEL3,
        A1_LOWALGAE,
        A2_HIGHALGAE
    }
}
