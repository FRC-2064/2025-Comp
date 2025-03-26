package frc.robot.Subsystems.Arm.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Enums.ArmState;

public class KArmSubsystem extends SubsystemBase {
    private TalonFX leader;
    private TalonFX follower;

    private TalonFXConfiguration leaderConfig;

    private CANdi absEncoder;

    private double armTarget;
    private double armAngle;

    private ArmState state = ArmState.STATIONARY;

    public KArmSubsystem() {
        leader = new TalonFX(ArmConstants.ARM_LEADER_ID);
        follower = new TalonFX(ArmConstants.ARM_FOLLOWER_ID);

        follower.setControl(new Follower(ArmConstants.ARM_LEADER_ID, true));

        var slot0 = leaderConfig.Slot0;
        slot0.kP = 5;
        slot0.kI = 0;
        slot0.kD = 0.1;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;

        var mmc = leaderConfig.MotionMagic;
        mmc.MotionMagicCruiseVelocity = 80;
        mmc.MotionMagicAcceleration = 160;
        mmc.MotionMagicJerk = 1600;
        

        leader.getConfigurator().apply(leaderConfig);
        

        absEncoder = new CANdi(56);
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
        // TODO: impliment kraken
    }

    public Double getMotorPositionAngle() {
        // TODO: impliment
        return leader.getPosition().getValueAsDouble();
    }
}
