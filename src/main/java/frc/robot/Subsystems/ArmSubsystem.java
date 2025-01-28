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
    private SparkMax intakeTop;
    private SparkMax intakeBottom;
    private SparkMax climbClamp;

    private SparkClosedLoopController armController;
    private SparkClosedLoopController climbClampController;

    private SparkFlexConfig armLeaderConfig;
    private SparkFlexConfig armFollowerConfig;
    private SparkFlexConfig climbClampConfig;

    private double armTargetAngle;
    private double armTarget;

    private double climbClampTargetAngle;
    
    private boolean newTarget = false;
    public boolean hasCoral = false;
    public boolean hasAlgae = false;

    double maxCurrent = 0.0;
    

    public ArmSubsystem() {
        //ARM
        armLeader = new SparkFlex(ArmConstants.ARM_LEADER_ID, MotorType.kBrushless);
        armFollower = new SparkFlex(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushless);
        
        armLeaderConfig = new SparkFlexConfig();
        armFollowerConfig = new SparkFlexConfig();
        
        armLeaderConfig
        .smartCurrentLimit(40)
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(10, 0, 0);
        
        armLeaderConfig
        .idleMode(IdleMode.kBrake)
        .closedLoop.maxMotion
        .maxVelocity(20)
        .maxAcceleration(20)
        .allowedClosedLoopError(0.01)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        armFollowerConfig.follow(ArmConstants.ARM_LEADER_ID, true);
        
        armLeader.configure(armLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollower.configure(armFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armController = armLeader.getClosedLoopController();
        
        //INTAKE
        intakeTop = new SparkMax(ArmConstants.INTAKE_TOP_ID, MotorType.kBrushless);
        intakeBottom = new SparkMax(ArmConstants.INTAKE_BOTTOM_ID, MotorType.kBrushless);
        
        //CLIMB
        climbClamp = new SparkMax(ArmConstants.CLIMB_ID, MotorType.kBrushless);

        climbClampConfig = new SparkFlexConfig();
        climbClampConfig
        .smartCurrentLimit(20)
        .closedLoop
        .pid(1, 0, 0)
        // .outputRange(0, 0.21)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        //CHECK IF NEEDS TO BE INVERTED OR NOT
        climbClampConfig.absoluteEncoder.inverted(true);

        climbClamp.configure(climbClampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbClampController = climbClamp.getClosedLoopController();



    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm target angle", armTarget);
        SmartDashboard.putNumber("arm angle", (armLeader.getAbsoluteEncoder().getPosition()));
        SmartDashboard.putBoolean("follower is follower", armFollower.isFollower());
        SmartDashboard.putNumber("Intake current", intakeTop.getOutputCurrent());

        

    }
        
    public void setTargetAngle(double angle) {
        armTarget = angle/360;
        armController.setReference(armTarget, ControlType.kPosition);
    }

    public void intakeCoral() {
        intakeTop.set(-0.5);
        intakeBottom.set(-0.5);
        updateGamePieceState();
    }

    public void outtakeCoral() {
        intakeTop.set(0.5);
        intakeBottom.set(0.5);
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

    private void updateGamePieceState() {
    double currentOutput = intakeTop.getOutputCurrent();

    if (currentOutput > 22) {
        if (!hasCoral) {
            hasCoral = true;
            SmartDashboard.putBoolean("Has Coral", hasCoral);
            stopIntakeMotors();
        }
    } else {
        
    }
}




    public void setClimbClampAngle(double angle){
        climbClampController.setReference(angle/360, ControlType.kPosition);
    }




}
