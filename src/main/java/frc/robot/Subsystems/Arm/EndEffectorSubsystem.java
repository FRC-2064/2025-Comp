package frc.robot.Subsystems.Arm;

import java.util.EnumMap;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DistanceSensorUSB;
import frc.robot.Utils.Constants.EndEffectorConstants;
import frc.robot.Utils.ControlBoard.ControlBoardHelpers;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax top;
    private SparkMax bottom;
    private DistanceSensorUSB tof;
    private SparkFlexConfig topConfig;
    private SparkFlexConfig bottomConfig;
    private EndEffectorState state = EndEffectorState.STOPPED;

    private DigitalInput endEffectorLimitSwitch;

    public boolean hasCoral = false;

    private final EnumMap<EndEffectorState, Runnable> stateActions;

    public EndEffectorSubsystem() {
        top = new SparkMax(EndEffectorConstants.EE_TOP_ID, MotorType.kBrushless);
        bottom = new SparkMax(EndEffectorConstants.EE_BOTTOM_ID, MotorType.kBrushless);

        topConfig = new SparkFlexConfig();
        bottomConfig = new SparkFlexConfig();

        endEffectorLimitSwitch = new DigitalInput(0);

        tof = new DistanceSensorUSB();

        stateActions = new EnumMap<>(EndEffectorState.class);
        stateActions.put(EndEffectorState.INTAKING_CORAL, this::intakeCoral);
        stateActions.put(EndEffectorState.OUTTAKING_CORAL, this::outtakeCoral);
        stateActions.put(EndEffectorState.INTAKING_ALGAE, this::intakeAlgae);
        stateActions.put(EndEffectorState.OUTTAKING_ALGAE, this::outtakeAlgae);
        stateActions.put(EndEffectorState.REMOVING_HIGH_ALGAE, this::removeHighAlgae);
        stateActions.put(EndEffectorState.REMOVING_LOW_ALGAE, this::removeLowAlgae);
        stateActions.put(EndEffectorState.OUTTAKING_PEG, this::OuttakePeg);
        stateActions.put(EndEffectorState.STOPPED, this::stop);


        topConfig
            .smartCurrentLimit(40);

        bottomConfig
            .smartCurrentLimit(40);

        top.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottom.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    @Override
    public void periodic() {
        tof.periodicUpdate();
        hasCoral = endEffectorLimitSwitch.get();
        // stopWithCoral();
        ControlBoardHelpers.setHasCoral(hasCoral);
        SmartDashboard.putString("Logging/EE/State", getState().toString());
        SmartDashboard.putNumber("Logging/DistanceSensor", tof.getDistance());
        //System.out.println(tof.getDistance());
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

    public Translation2d getGamePieceOffset() {
        double currentReading = tof.getDistance();
        return new Translation2d((-(EndEffectorConstants.EE_BASE_OFFSET - currentReading)/1000), 0.0);
    }

    private void stopWithCoral() { 
        if (state != EndEffectorState.INTAKING_CORAL) {
            return;
        }

        if (hasCoral) {
            setState(EndEffectorState.STOPPED);
        }
    }

    public void stop() {
        top.set(0.0);
        bottom.set(0.0);
    }

    public void intakeCoral(){
        top.set(0.5);
        bottom.set(0.5);
    }

    public void outtakeCoral(){
        top.set(-0.35);
        bottom.set(-0.35);
    }

    private void intakeAlgae(){
        top.set(0.0);
        bottom.set(0.5);
    }

    private void outtakeAlgae(){
        top.set(0.5);
        bottom.set(0.0);
    }

    private void removeHighAlgae(){
        top.set(-0.75);
        bottom.set(0.0);
    }

    private void removeLowAlgae(){
        top.set(0.0);
        bottom.set(-0.75);
    }

    private void OuttakePeg(){
        top.set(-0.5);
        bottom.set(-0.5);
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
}

