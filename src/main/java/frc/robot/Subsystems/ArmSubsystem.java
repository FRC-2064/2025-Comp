package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkFlex armLeader;
    private SparkFlex armFollower;
    private SparkMax climbClamp;

    private SparkClosedLoopController armController;
    private SparkClosedLoopController climbClampController;

    private SparkFlexConfig armLeaderConfig;
    private SparkFlexConfig armFollowerConfig;
    private SparkFlexConfig climbClampConfig;

    private double armTarget;

    public boolean hasCoral = false;
    public boolean hasAlgae = false;

    private CLAMP_STATE clampState = CLAMP_STATE.UN_CLAMPED;

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
                .allowedClosedLoopError(0.05)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        armFollowerConfig.follow(ArmConstants.ARM_LEADER_ID, true);

        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armController = armLeader.getClosedLoopController();

        // CLIMB
        // climbClamp = new SparkMax(ArmConstants.CLIMB_ID, MotorType.kBrushless);

        // climbClampConfig = new SparkFlexConfig();
        // climbClampConfig
        //         .idleMode(IdleMode.kBrake)
        //         .smartCurrentLimit(20).closedLoop
        //         .pid(5, 0, 0)
        //         .positionWrappingEnabled(true)
        //         .positionWrappingInputRange(0, 1)
        //         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // climbClampConfig.absoluteEncoder.inverted(false);
        // climbClamp.configure(climbClampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //climbClampController = climbClamp.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm angle", (armLeader.getAbsoluteEncoder().getPosition() * 360));
    }

    public void setTargetAngle(double angle) {
        armTarget = angle / 360;
        armController.setReference(armTarget, ControlType.kPosition);
    }

    // public void toggleClamp() {
    //     switch (clampState) {
    //         case CLAMPED:
    //             climbClampController.setReference(ArmConstants.HOME_CLAMP_VAL, ControlType.kPosition);
    //             clampState = CLAMP_STATE.UN_CLAMPED;
    //             break;

    //         case UN_CLAMPED:
    //             climbClampController.setReference(ArmConstants.CLIMB_CLAMP_VAL, ControlType.kPosition);
    //             clampState = CLAMP_STATE.CLAMPED;
    //             break;
    //     }
    // }

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

    private enum CLAMP_STATE {
        CLAMPED,
        UN_CLAMPED
    }

}
