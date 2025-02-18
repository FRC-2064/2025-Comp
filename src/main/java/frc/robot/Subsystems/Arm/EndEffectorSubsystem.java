package frc.robot.Subsystems.Arm;

import java.util.EnumMap;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.TOF;
import frc.robot.Utils.Constants.EndEffectorConstants;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax top;
    private SparkMax bottom;
    private TOF tof;
    private EndEffectorState state = EndEffectorState.STOPPED;

    private boolean hasCoral = false;
    private boolean HasAlgae = false;

    private final EnumMap<EndEffectorState, Runnable> stateActions;

    public EndEffectorSubsystem() {
        top = new SparkMax(EndEffectorConstants.EE_TOP_ID, MotorType.kBrushless);
        bottom = new SparkMax(EndEffectorConstants.EE_BOTTOM_ID, MotorType.kBrushless);
        tof = new TOF(EndEffectorConstants.TOF_PORT);

        stateActions = new EnumMap<>(EndEffectorState.class);
        stateActions.put(EndEffectorState.INTAKING_CORAL, this::intakeCoral);
        stateActions.put(EndEffectorState.OUTTAKING_CORAL, this::outtakeCoral);
        stateActions.put(EndEffectorState.INTAKING_ALGAE, this::intakeAlgae);
        stateActions.put(EndEffectorState.OUTTAKING_ALGAE, this::outtakeAlgae);
        stateActions.put(EndEffectorState.REMOVING_HIGH_ALGAE, this::removeHighAlgae);
        stateActions.put(EndEffectorState.REMOVING_LOW_ALGAE, this::removeLowAlgae);
        stateActions.put(EndEffectorState.STOPPED, this::stop);
    }

    @Override
    public void periodic() {
        stopWithCoral();
        ControlBoardHelpers.setHasAlgae(HasAlgae);
        ControlBoardHelpers.setHasCoral(hasCoral);
        SmartDashboard.putString("Logging/EE/State", getState().toString());
    }

    public void setState(EndEffectorState newState) {
        if (state == newState) {
            return;
        }

        state = newState;
        Runnable action = stateActions.get(newState);
        if (action != null) {
            action.run();
        }
    }

    public EndEffectorState getState() {
        return state;
    }

    public double getGamePieceOffset() {
        double currentReading = tof.getDistance();
        return EndEffectorConstants.EE_BASE_OFFSET - currentReading;
    }

    private void stopWithCoral() { 
        if (state != EndEffectorState.INTAKING_CORAL) {
            return;
        }

        if (hasCoral) {
            setState(EndEffectorState.STOPPED);
        }
    }

    private void stop() {
        top.set(0.0);
        bottom.set(0.0);
    }

    private void intakeCoral(){
        top.set(-0.5);
        bottom.set(-0.5);
    }

    private void outtakeCoral(){
        top.set(0.5);
        bottom.set(0.5);
    }

    private void intakeAlgae(){
        top.set(0.0);
        bottom.set(0.5);
    }

    private void outtakeAlgae(){
        top.set(0.0);
        bottom.set(-0.5);
    }

    private void removeHighAlgae(){
        top.set(0.5);
        bottom.set(0.0);
    }

    private void removeLowAlgae(){
        top.set(0.0);
        bottom.set(0.5);
    }



    public enum EndEffectorState {
        INTAKING_CORAL,
        OUTTAKING_CORAL,
        INTAKING_ALGAE,
        OUTTAKING_ALGAE,
        REMOVING_HIGH_ALGAE,
        REMOVING_LOW_ALGAE,
        STOPPED
    }
}

