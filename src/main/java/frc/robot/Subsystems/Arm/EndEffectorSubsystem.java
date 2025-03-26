package frc.robot.Subsystems.Arm;

import java.util.EnumMap;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.EndEffectorConstants;
import frc.robot.Utils.Enums.EndEffectorState;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax top;
    private SparkMax left;
    private SparkMax right;

    private SparkMaxConfig topConfig = new SparkMaxConfig();
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private CANrange tof;

    private EndEffectorState state = EndEffectorState.STOPPED;

    private final EnumMap<EndEffectorState, Runnable> stateActions;

    public boolean hasCoral = false;

    public EndEffectorSubsystem() {
        tof = new CANrange(EndEffectorConstants.EE_CANRANGE_ID);

        stateActions = new EnumMap<>(EndEffectorState.class);
        stateActions.put(EndEffectorState.INTAKING_CORAL, this::intakeCoral);
        stateActions.put(EndEffectorState.OUTTAKING_CORAL, this::outtakeCoral);
        stateActions.put(EndEffectorState.INTAKING_ALGAE, this::intakeAlgae);
        stateActions.put(EndEffectorState.OUTTAKING_ALGAE, this::outtakeAlgae);
        stateActions.put(EndEffectorState.REMOVING_HIGH_ALGAE, this::removeHighAlgae);
        stateActions.put(EndEffectorState.REMOVING_LOW_ALGAE, this::removeLowAlgae);
        stateActions.put(EndEffectorState.OUTTAKING_PEG, this::OuttakePeg);
        stateActions.put(EndEffectorState.STOPPED, this::stop);

        top = new SparkMax(EndEffectorConstants.EE_TOP_ID, MotorType.kBrushless);
        left = new SparkMax(EndEffectorConstants.EE_LEFT_ID, MotorType.kBrushless);
        right = new SparkMax(EndEffectorConstants.EE_RIGHT_ID, MotorType.kBrushless);

        topConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);
        leftConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true);
        rightConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(false);

        top.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        left.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        hasCoral = tof.getIsDetected().getValue();
        ControlBoardHelpers.setHasCoral(hasCoral);
        SmartDashboard.putString("Logging/EE/State", getState().toString());
        
        // if (hasCoral == true && state == EndEffectorState.INTAKING_CORAL) {
        //     setState(EndEffectorState.STOPPED);
        // }
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

    private void stop() {
        top.set(0.0);
        left.set(0.0);
        right.set(0.0);
    }

    private void intakeCoral() {
        // if (hasCoral) {
        //     return;
        // }
        top.set(0.35);
        left.set(0.75);
        right.set(0.25);
    }

    private void outtakeCoral() {
        top.set(-1.0);
        left.set(-1.0);
        right.set(-1.0);
    }

    private void intakeAlgae(){
        top.set(-0.5);
        left.set(0.0);
        right.set(0.0);
    }

    private void outtakeAlgae()  {
        top.set(0.5);
        left.set(0.0);
        right.set(0.0);
    }

    private void removeHighAlgae()  {
        top.set(-0.75);
        left.set(0.0);
        right.set(0.0);
    }

    private void removeLowAlgae()  {
        top.set(-0.75);
        left.set(0.0);
        right.set(0.0);
    }

    private void OuttakePeg()  {
        top.set(-0.75);
        left.set(-0.75);
        right.set(-0.75);
    }


}
