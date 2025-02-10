package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax top;
    private SparkMax bottom;
    private EndEffectorState currentState = EndEffectorState.STOPPED;
    private EndEffectorState desiredState = EndEffectorState.STOPPED;

    public EndEffectorSubsystem() {
        top = new SparkMax(EndEffectorConstants.EE_TOP_ID, MotorType.kBrushless);
        bottom = new SparkMax(EndEffectorConstants.EE_BOTTOM_ID, MotorType.kBrushless);
    }

    public void setState(EndEffectorState state) {
        desiredState = state;
    }

    public EndEffectorState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        if (currentState != desiredState) {
            manageState();
        }
        SmartDashboard.putString("Logging/EE/State", currentState.toString());
    }

    private void manageState() {
        switch (desiredState) {
            case INTAKING_CORAL:
                intakeCoral();
                break;
            case OUTTAKING_CORAL:
                outtakeCoral();
                break;
            case INTAKING_ALGAE:
                intakeAlgae();
                break;
            case OUTTAKING_ALGAE:
                outtakeAlgae();
                break;
            case REMOVING_HIGH_ALGAE:
                removeHighAlgae();
                break;
            case REMOVING_LOW_ALGAE:
                removeLowAlgae();
                break;
            case STOPPED:
                stop();
                break;
        }
    }

    private void stop() {
        top.set(0.0);
        bottom.set(0.0);
        currentState = EndEffectorState.STOPPED;
    }

    private void intakeCoral(){
        top.set(-0.5);
        bottom.set(-0.5);
        currentState = EndEffectorState.INTAKING_CORAL;
    }

    private void outtakeCoral(){
        top.set(0.5);
        bottom.set(0.5);
        currentState = EndEffectorState.OUTTAKING_CORAL;
    }

    private void intakeAlgae(){
        top.set(0.0);
        bottom.set(0.5);
        currentState = EndEffectorState.INTAKING_ALGAE;
    }

    private void outtakeAlgae(){
        top.set(0.0);
        bottom.set(-0.5);
        currentState = EndEffectorState.OUTTAKING_ALGAE;
    }

    private void removeHighAlgae(){
        top.set(0.5);
        bottom.set(0.0);
        currentState = EndEffectorState.REMOVING_HIGH_ALGAE;
    }

    private void removeLowAlgae(){
        top.set(0.0);
        bottom.set(0.5);
        currentState = EndEffectorState.REMOVING_LOW_ALGAE;
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

