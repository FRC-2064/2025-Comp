package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ClampConstants;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;

public class ClampSubsystem extends SubsystemBase {
    private SparkMax clamp;
    private SparkClosedLoopController clampController;
    private SparkMaxConfig clampConfig;

    private ClampState currentState = ClampState.OPEN;
    private ClampState desiredState = ClampState.OPEN;

    public ClampSubsystem() {
        clamp = new SparkMax(ClampConstants.CLAMP_ID, MotorType.kBrushless);

        clampConfig = new SparkMaxConfig();
        clampConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20).closedLoop
                .pid(5, 0, 0)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 1)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        clampConfig.absoluteEncoder.inverted(false);
        clamp.configure(clampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clampController = clamp.getClosedLoopController();
    }

    public void setState(ClampState state) {
        desiredState = state;
    }

    public ClampState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        if (currentState != desiredState) {
            manageState();
        }
        SmartDashboard.putString("Logging/Clamp/State", currentState.toString());

    }

    private void manageState() {
        switch (desiredState) {
            case OPEN:
                open();
                break;
            case CLOSED:
                close();
                break;
            default:
                break;
        }
    }

    private void open() {
        clampController.setReference(ClampConstants.CLAMP_OPEN_VAL, ControlType.kPosition);
        currentState = ClampState.OPEN;
        ControlBoardHelpers.setClamped(false);
    }

    private void close() {
        clampController.setReference(ClampConstants.CLAMP_CLOSED_VAL, ControlType.kPosition);
        currentState = ClampState.CLOSED;
        ControlBoardHelpers.setClamped(true);
    }


    public enum ClampState {
        OPEN,
        CLOSED,
        CLAMPED
    }
}
