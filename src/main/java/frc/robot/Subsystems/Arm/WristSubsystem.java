package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.Enums.WristState;

public class WristSubsystem extends SubsystemBase {
    private TalonFX wrist = new TalonFX(WristConstants.WRIST_ID);
    private CANcoder encoder = new CANcoder(WristConstants.WRIST_ENCODER_ID);

    private TalonFXConfiguration wristConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage wristControl = new MotionMagicVoltage(0.0);
    private double wristAngle;
    private double wristTarget;

    private WristState state = WristState.STATIONARY;

    private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public WristSubsystem() {
        var slot0 = wristConfig.Slot0;
        slot0.kP = 80;
        slot0.kD = 0.0;
        slot0.kS = 0.15;
        slot0.kV = 0.65;
        slot0.kA = 0.03;
        slot0.kG = 0.4;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var mmc = wristConfig.MotionMagic;
        mmc.MotionMagicCruiseVelocity = 5;
        mmc.MotionMagicAcceleration = 4;

        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.Feedback.withRotorToSensorRatio(WristConstants.WRIST_GEAR_RATIO).withFusedCANcoder(encoder);
        
        wrist.getConfigurator().apply(wristConfig);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    }

    @Override
    public void periodic() {
        wristAngle = (wrist.getPosition().getValueAsDouble() + 0.5) * 360;
        if (Math.abs(wristAngle - wristTarget) < WristConstants.ALLOWED_ERROR_DEGREES) {
            state = WristState.STATIONARY;
        } else {
            state = WristState.MOVING;
        }
        SmartDashboard.putString("Logging/Wrist/State", state.toString());
        SmartDashboard.putNumber("Logging/Wrist/Angle", wristAngle);
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public WristState getState() {
        return state;
    }

    public double getTargetWristAngle() {
        return wristTarget;
    }

    public void toggleWristBrake() {
        switch (neutralMode) {
            case Brake:
                neutralMode = NeutralModeValue.Coast;
                wrist.setNeutralMode(NeutralModeValue.Coast);
                break;
        
            case Coast:
                neutralMode = NeutralModeValue.Brake;
                wrist.setNeutralMode(NeutralModeValue.Brake);
                break;
        }
    }

    public void setTargetAngle(double angle) {
        double positionTarget = (angle / 360) - 0.5; // -0.5 TO 0.5
        wristTarget = positionTarget;
        wristControl.withPosition(positionTarget);
        ControlRequest wcr = wristControl;

        wrist.setControl(wcr);
    }

}
