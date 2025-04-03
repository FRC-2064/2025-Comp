package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ClampConstants;
import frc.robot.Utils.Enums.ClimbState;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax clamp;
    private SparkClosedLoopController clampController;
    private SparkMaxConfig clampConfig;

    private SparkFlex winch;
    private SparkFlexConfig winchConfig;

    private ClimbState currentState = ClimbState.OPEN;

    public ClimbSubsystem() {
        clamp = new SparkMax(ClampConstants.CLAMP_ID, MotorType.kBrushless);
        winch = new SparkFlex(ClampConstants.WINCH_ID, MotorType.kBrushless);

        clampConfig = new SparkMaxConfig();
        winchConfig = new SparkFlexConfig();

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

        winchConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        winchConfig.inverted(true);

        winch.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public ClimbState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Logging/Clamp/State", currentState.toString());
        SmartDashboard.putNumber("Logging/Clamp/ClampAngle", clamp.getAbsoluteEncoder().getPosition());
        // SmartDashboard.putNumber("Logging/Clamp/ClampAngle", clamp)

    }

    public void toggleClamp() {
        switch (currentState) {
            case OPEN:
                close();
                break;

            case CLOSED:
                open();
                break;
            default:
                break;
        }
    }

    private void open() {
        clampController.setReference(ClampConstants.CLAMP_OPEN_VAL, ControlType.kPosition);
        currentState = ClimbState.OPEN;
        ControlBoardHelpers.setClamped(false);
    }

    private void close() {
        clampController.setReference(ClampConstants.CLAMP_CLOSED_VAL, ControlType.kPosition);
        currentState = ClimbState.CLOSED;
        ControlBoardHelpers.setClamped(true);
    }

    // negative speed is winch in!!!
    public void winchIn() {
        if (DriverStation.getMatchTime() > 30) {
            return;
        }
        winch.set(0.25);
    }

    public void winchStop() {
        winch.set(0);
    }

}
