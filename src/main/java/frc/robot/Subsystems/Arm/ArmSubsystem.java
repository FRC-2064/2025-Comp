package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkFlex armLeader;
    private SparkFlex armFollower;

    private SparkClosedLoopController armController;

    private SparkFlexConfig armLeaderConfig;
    private SparkFlexConfig armFollowerConfig;

    private double armTarget;
    private double armAngle;

    private ArmState currentState = ArmState.STATIONARY;


    public ArmSubsystem() {
        // ARM
        armLeader = new SparkFlex(ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
        armFollower = new SparkFlex(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);

        armLeaderConfig = new SparkFlexConfig();
        armFollowerConfig = new SparkFlexConfig();

        armLeaderConfig
                .smartCurrentLimit(40)
                .inverted(true)
                .idleMode(IdleMode.kBrake).closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(10, 0, 0)
                .positionWrappingEnabled(false);

        armLeaderConfig
                .idleMode(IdleMode.kBrake).closedLoop.maxMotion
                .maxVelocity(8000)
                .maxAcceleration(4000)
                .allowedClosedLoopError(ArmConstants.ALLOWED_ERROR_DEGREES / ArmConstants.DEGREES_PER_ROTATION)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        armFollowerConfig.follow(ArmConstants.ARM_LEADER_ID, true);

        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = armLeader.getClosedLoopController();

    }

    @Override
    public void periodic() {
        armAngle = armLeader.getAbsoluteEncoder().getPosition() * ArmConstants.DEGREES_PER_ROTATION;
        if (Math.abs(armAngle - armTarget) < ArmConstants.ALLOWED_ERROR_DEGREES) {
            currentState = ArmState.STATIONARY;
        } else {
            currentState = ArmState.MOVING;
        }
        SmartDashboard.putString("Logging/Arm/State", currentState.toString());
        SmartDashboard.putNumber("Logging/Arm/Angle", armAngle);
    }

    public double getArmAngle() {
        return armAngle;
    }

    public ArmState getState() {
        return currentState;
    }

    public void setTargetAngle(double angle) {
        if (armTarget == angle) {
            return;
        }
        armTarget = angle;
        double normalizedTarget = angle / ArmConstants.DEGREES_PER_ROTATION;
        armController.setReference(normalizedTarget, ControlType.kPosition);
    }

    public void armToggleCoast() {
        switch (armLeader.configAccessor.getIdleMode()) {
            case kBrake:
                armLeaderConfig.idleMode(IdleMode.kCoast);
                armFollowerConfig.idleMode(IdleMode.kCoast);

                armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
                break;
            case kCoast:
                armLeaderConfig.idleMode(IdleMode.kBrake);
                armFollowerConfig.idleMode(IdleMode.kBrake);

                armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
                break;
        }
    }

    public enum ArmState {
        MOVING,
        STATIONARY
    }

}
