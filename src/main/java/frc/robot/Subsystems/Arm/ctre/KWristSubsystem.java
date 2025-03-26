package frc.robot.Subsystems.Arm.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.Enums.WristState;

public class KWristSubsystem extends SubsystemBase {
    private TalonFX wrist = new TalonFX(WristConstants.WRIST_ID);;

    private TalonFXConfiguration wristConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage wristControl = new MotionMagicVoltage(0.0);
    private double wristAngle;
    private double wristTarget;

    private WristState state = WristState.STATIONARY;


    public KWristSubsystem(CANdi candi) {
        var slot0 = wristConfig.Slot0;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var mmc = wristConfig.MotionMagic;
        mmc.MotionMagicCruiseVelocity = 80;
        mmc.MotionMagicAcceleration = 200;
        mmc.MotionMagicJerk = 300;

        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.Feedback.withRotorToSensorRatio(WristConstants.WRIST_GEAR_RATIO).withFusedCANdiPwm1(candi);
        
        wrist.getConfigurator().apply(wristConfig);
    
    }

    @Override
    public void periodic() {
        wristAngle = getMotorPositionAngle();
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

    public void setTargetAngle(double angle) {
        wristControl.withPosition(angle / 360);
        ControlRequest wcr = wristControl;

        wrist.setControl(wcr);
    }

    public Double getMotorPositionAngle() {
        // TODO: impliment
        return wrist.getPosition().getValueAsDouble();
    }



}
