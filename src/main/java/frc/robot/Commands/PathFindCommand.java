package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlBoard.ControlBoardUtils;
import frc.robot.ControlBoard.ScoreOutput;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class PathFindCommand extends Command {
    SwerveSubsystem swerveDrive;
    ArmSubsystem arm;
    WristSubsystem wrist;
    String goal;

    public PathFindCommand(SwerveSubsystem swerveDrive, ArmSubsystem arm, WristSubsystem wrist, String goal) {
        this.swerveDrive = swerveDrive;
        this.arm = arm;
        this.wrist = wrist;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        ScoreOutput output = null;
        switch (goal) {
            case "SCORE":
                output = ControlBoardUtils.getScorePath(swerveDrive.getSwerveDrive().getOdometryHeading().getDegrees());
                break;

            case "FEEDER":
                output = ControlBoardUtils.getScorePath(swerveDrive.getSwerveDrive().getOdometryHeading().getDegrees());
                break;

            case "CAGE":
                output = ControlBoardUtils.getScorePath(swerveDrive.getSwerveDrive().getOdometryHeading().getDegrees());
                break;

            default:
                break;
        }

        if (output != null) {
            SmartDashboard.putString("Logging/outputfromcmd", output.toString());
            swerveDrive.pathfindToPath(output.path);
            arm.setTargetAngle(output.armAngle);
            wrist.setWristAngle(output.wristAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

