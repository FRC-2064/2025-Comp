package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ArmSubsystem;

public class RemoveAlgaeHighCMD extends Command {
    private ArmSubsystem arm;

    public RemoveAlgaeHighCMD(ArmSubsystem armsub) {
        arm = armsub;
    }

    @Override
    public void initialize() {
        arm.setTargetAngle(ArmConstants.HIGH_ALGAE_REMOVAL_ANGLE);
        arm.removeAlgaeHigh();
    }

    @Override
    public void end(boolean interupted) {
        arm.setTargetAngle(ArmConstants.HOME_ANGLE);
        arm.stopIntakeMotors();
    }

}
