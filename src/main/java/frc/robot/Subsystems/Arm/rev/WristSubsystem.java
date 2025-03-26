package frc.robot.Subsystems.Arm.rev;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.WristConstants;
import frc.robot.Utils.Enums.WristState;

public class WristSubsystem extends SubsystemBase {
    SparkMax wristMotor;

    private SparkClosedLoopController wristController;

    private SparkMaxConfig wristConfig;

    private double wristAngle;
    private double wristTarget;

    private WristState currentState = WristState.STATIONARY;

    public WristSubsystem() {
        wristMotor = new SparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);
        wristConfig = new SparkMaxConfig();

        wristConfig
                .smartCurrentLimit(20)
                .inverted(false)
                .idleMode(IdleMode.kBrake).closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(false)
                .pid(2, 0, 1)
                .maxMotion
                .maxVelocity(5676)
                .maxAcceleration(10000)
                .allowedClosedLoopError(WristConstants.ALLOWED_ERROR_DEGREES / WristConstants.DEGREES_PER_ROTATION);

        wristConfig.absoluteEncoder.inverted(true);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        wristController = wristMotor.getClosedLoopController();

    }

    @Override
    public void periodic() {
        wristAngle = wristMotor.getAbsoluteEncoder().getPosition() * WristConstants.DEGREES_PER_ROTATION;
        if (Math.abs(wristAngle - wristTarget) < WristConstants.ALLOWED_ERROR_DEGREES) {
            currentState = WristState.STATIONARY;
        } else {
            currentState = WristState.MOVING;
        }

        SmartDashboard.putString("Logging/Wrist/State", currentState.toString());
        SmartDashboard.putNumber("Logging/Wrist/Angle", wristAngle);
        SmartDashboard.putNumber("Logging/Wrist/DesiredAngle", wristTarget);
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public WristState getWristState() {
        return currentState;
    }

    public void setTargetAngle(double angle) {
        if (wristTarget == angle) {
            return;
        }
        wristTarget = angle;
        double normalizedTarget = angle / WristConstants.DEGREES_PER_ROTATION;
        wristController.setReference(normalizedTarget, ControlType.kPosition);
    }

    public void wristToggleCoast() {
        switch (wristMotor.configAccessor.getIdleMode()) {
            case kBrake:
                wristConfig.idleMode(IdleMode.kCoast);
                break;
            case kCoast:
                wristConfig.idleMode(IdleMode.kBrake);
                break;
            }
            
            wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


}