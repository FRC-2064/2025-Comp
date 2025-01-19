package frc.robot.Subsystems.SwerveDrive;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShovelSubsystem extends SubsystemBase{
    private SparkMax shovelMotor;
    private SparkClosedLoopController motionController;
    private SparkMaxConfig shovelConfig;

    public double shovelTargetAngle;
    public double shovelTarget;
    private boolean referenceSet = true;

    public ShovelSubsystem(){
        shovelMotor = new SparkMax(Constants.ShovelConstants.shovelRotatorID, SparkLowLevel.MotorType.kBrushless);
        motionController = shovelMotor.getClosedLoopController();

        shovelConfig = new SparkMaxConfig();
        shovelConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0.0, 0.1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        shovelConfig.closedLoop.maxMotion
        .maxVelocity(20)
        .maxAcceleration(20)
        .allowedClosedLoopError(0.01)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
        shovelConfig.absoluteEncoder.inverted(true);
        shovelConfig.inverted(true);

        shovelMotor.configure(shovelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setShovelAngle(double target){
        shovelTargetAngle = target;
        shovelTarget = target / 360;
        referenceSet = false;
    }

    @Override
    public void periodic(){
        if (!referenceSet) {
            motionController.setReference(shovelTargetAngle, ControlType.kPosition);
            referenceSet = true;
        }
    }
}