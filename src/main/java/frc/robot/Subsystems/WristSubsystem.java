package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    //SparkMax wristMotor;
    SparkMax intakeTop;
    SparkMax intakeBottom;

    private SparkClosedLoopController wristController;
    
    private SparkMaxConfig wristConfig;

    public WristSubsystem() {
        //wristMotor = new SparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);
        intakeTop = new SparkMax(WristConstants.INTAKE_TOP_ID, MotorType.kBrushless);
        intakeBottom = new SparkMax(WristConstants.INTAKE_BOTTOM_ID, MotorType.kBrushless);

        wristConfig = new SparkMaxConfig();

        wristConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0, 0);

        //wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //wristController = wristMotor.getClosedLoopController();


    }


    @Override
    public void periodic(){
        
    }

    public void setWristAngle(double angle) {
        wristController.setReference(angle/360, ControlType.kPosition);
    }

    public void intakeCoral() {
        intakeTop.set(-0.5);
        intakeBottom.set(-0.5);
    }

    public void outtakeCoral() {
        intakeTop.set(0.20);
        intakeBottom.set(0.20);
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


