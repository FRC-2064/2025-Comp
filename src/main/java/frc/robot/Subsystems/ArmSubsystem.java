package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkFlex armLeader;
    private SparkFlex armFollower;
    private SparkMax intakeTop;
    private SparkMax intakeBottom;
    private SparkMax climbClamp;

    private SparkClosedLoopController armController;

    private SparkFlexConfig armLeaderConfig;
    private SparkFlexConfig armFollowerConfig;

    private double armTargetAngle;
    private double armTarget;
    
    private boolean newTarget = false;
    public boolean hasCoral = false;
    public boolean hasAlgae = false;
    

    public ArmSubsystem() {
        //ARM
        armLeader = new SparkFlex(ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
        armFollower = new SparkFlex(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);
        
        armLeaderConfig = new SparkFlexConfig();
        armFollowerConfig = new SparkFlexConfig();
        
        armLeaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0, 0, 0);
        
        armLeaderConfig.closedLoop.maxMotion
        .maxVelocity(20)
        .maxAcceleration(20)
        .allowedClosedLoopError(0.01)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        armFollowerConfig.follow(ArmConstants.ARM_LEADER_ID)
        .inverted(true);
        
        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armController = armLeader.getClosedLoopController();
        
        //INTAKE
        intakeTop = new SparkMax(ArmConstants.INTAKE_TOP_ID, MotorType.kBrushless);
        intakeBottom = new SparkMax(ArmConstants.INTAKE_BOTTOM_ID, MotorType.kBrushless);
        
        //CLIMB
        climbClamp = new SparkMax(ArmConstants.CLIMB_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        if (newTarget) {
            armController.setReference(armTarget, ControlType.kPosition);
            newTarget = false;
        }
    }

    public void setTargetAngle(double angle) {
        armTargetAngle = angle;
        armTarget = angle / 360;
        newTarget = true;
    }

    public void intakeCoral() {
        intakeTop.set(1);
        intakeBottom.set(-1);
    }

    public void outtakeCoral() {
        intakeTop.set(-1);
        intakeBottom.set(1);
    }

    public void removeAlgaeLow() {
        intakeTop.set(-1);
    }
    
    public void removeAlgaeHigh() {
        intakeTop.set(1);
    }

    public void stopIntakeMotors() {
        intakeTop.set(0);
        intakeBottom.set(0);
    }

}
