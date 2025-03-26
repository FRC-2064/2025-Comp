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
        INTAKING_CORAL,
        OUTTAKING_CORAL,
        INTAKING_ALGAE,
        OUTTAKING_ALGAE,
        REMOVING_HIGH_ALGAE,
        REMOVING_LOW_ALGAE,
        OUTTAKING_PEG,
        STOPPED
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
