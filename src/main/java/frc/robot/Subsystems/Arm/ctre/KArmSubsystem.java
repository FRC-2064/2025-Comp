package frc.robot.Subsystems.Arm.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Enums.ArmState;

public class KArmSubsystem extends SubsystemBase {
    private TalonFX leader = new TalonFX(ArmConstants.ARM_LEADER_ID);
    private TalonFX follower = new TalonFX(ArmConstants.ARM_FOLLOWER_ID);

    private final MotionMagicVoltage armControl = new MotionMagicVoltage(0.0);

    private TalonFXConfiguration leaderConfig = new TalonFXConfiguration();


    private double armTarget;
    private double armAngle;

    private ArmState state = ArmState.STATIONARY;

    public KArmSubsystem(CANdi candi) {
        follower.setControl(new Follower(ArmConstants.ARM_LEADER_ID, true));


        var slot0 = leaderConfig.Slot0;
        slot0.kP = 80;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        var mmc = leaderConfig.MotionMagic;
        mmc.MotionMagicCruiseVelocity = 80;
        mmc.MotionMagicAcceleration = 200;
        mmc.MotionMagicJerk = 400;

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leaderConfig.Feedback.withRotorToSensorRatio(ArmConstants.ARM_GEAR_RATIO).withFusedCANdiPwm2(candi);
        
        leader.getConfigurator().apply(leaderConfig);
    
    }

        @Override
    public void periodic() {
        armAngle = getMotorPositionAngle();
        if (Math.abs(armAngle - armTarget) < ArmConstants.ALLOWED_ERROR_DEGREES) {
            state = ArmState.STATIONARY;
        } else {
            state = ArmState.MOVING;
        }
        SmartDashboard.putString("Logging/Arm/State", state.toString());
        SmartDashboard.putNumber("Logging/Arm/Angle", armAngle);
    }

    public double getArmAngle() {
        return armAngle;
    }

    public ArmState getState() {
        return state;
    }

    public double getTargetArmAngle() {
        return armTarget;
    }

    public void setTargetAngle(double angle) {
        armControl.withPosition(angle / 360);
        ControlRequest acr = armControl;

        leader.setControl(acr);
    }

    public Double getMotorPositionAngle() {
        return leader.getPosition().getValueAsDouble();
    }
}
