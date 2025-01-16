package frc.robot.Subsystems.SwerveDrive;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShovelSubsystem extends SubsystemBase{
    private SparkAbsoluteEncoder shovelEncoder;
    private SparkMax shovelMotor;
    private SparkClosedLoopController maxPID;
    private SparkMaxConfig shovelConfig;

    public double shovelTargetAngle;
    public double shovelTarget;

    public ShovelSubsystem(){
        shovelMotor = new SparkMax(Constants.kShovelSubsystem.shovelRotatorID, SparkLowLevel.MotorType.kBrushless);
        shovelEncoder = shovelMotor.getAbsoluteEncoder();
        maxPID = shovelMotor.getClosedLoopController();
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
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        shovelConfig.absoluteEncoder.inverted(true);
        shovelConfig.inverted(true);
        shovelMotor.configure(shovelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void setShovelAngle(double target){
        shovelTargetAngle = target;
        shovelTarget = target / 360;
    }
    public double getShovelAngle(){
        return shovelEncoder.getPosition() * 360;
    }
    public void dump(){
        shovelTargetAngle = -30;
    }
    public void carry(){
        shovelTargetAngle = 45;
    }
    public void pickUp(){
        shovelTargetAngle = 15;
    }
    //public boolean hasCoral(double angle){
        //this.angle = getShovelAngle();
    //}

    @Override
    public void periodic(){
        SmartDashboard.getNumber("targetAngle", shovelTargetAngle);
        SmartDashboard.putNumber("EncoderVal", shovelEncoder.getPosition());
        maxPID.setReference(shovelTargetAngle / 360, ControlType.kPosition);
    }
}