package frc.robot.Utils;

public class Enums {
    public enum ArmState {
        MOVING,
        STATIONARY
    }

    public enum WristState {
        MOVING,
        STATIONARY
    }

    public enum ClimbState {
        OPEN,
        CLOSED,
        CLAMPED
    }

    public enum EndEffectorState {
        INTAKING_CORAL_GROUND,
        INTAKING_CORAL_FEEDER,
        OUTTAKING_TROUGH,
        OUTTAKING_LEVEL_2_FRONT,
        OUTTAKING_LEVEL_2_BACK,
        OUTTAKING_LEVEL_3,
        INTAKING_ALGAE,
        OUTTAKING_ALGAE,
        REMOVING_HIGH_ALGAE,
        REMOVING_LOW_ALGAE,
        STOPPED
    }

    public enum LEDState {
        OFF,
        LIGHTBAR,
        HAS_PIECE,
        NO_PIECE,
        CLIMBED
    }

    public enum RobotState {
        I_IDLE,
        C_CLIMBING,
        B_BRAKINGCLIMB,
        T_TRAVELING,
        P_PATHING,
        F_FEEDER,
        G_GROUND,
        S_SCORING,
        M_MANUAL,
        W_WIN
    }
}
