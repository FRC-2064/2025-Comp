package frc.robot.Utils.ControlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;

public class RobotConfiguration {
    public final Pose2d desiredEndPose;
    public final Pose2d desiredStartPose;
    public final double finalArmAngle;
    public final double travelArmAngle;
    public final double finalWristAngle;
    public final double travelWristAngle;
    public final EndEffectorState startEndEffectorState;
    public EndEffectorState finalEndEffectorState;

    
    public RobotConfiguration(
        Pose2d desiredEndPose,
        Pose2d desiredStartPose,
        double finalArmAngle, 
        double travelArmAngle, 
        double finalWristAngle,  
        double travelWristAngle,
        EndEffectorState startEndEffectorState,
        EndEffectorState finalEndEffectorState) {

        this.desiredEndPose = desiredEndPose;
        this.desiredStartPose = desiredStartPose;
        this.finalArmAngle = finalArmAngle;
        this.travelArmAngle = travelArmAngle;
        this.finalWristAngle = finalWristAngle;
        this.travelWristAngle = travelWristAngle;
        this.startEndEffectorState = startEndEffectorState;
        this.finalEndEffectorState = finalEndEffectorState;
    }

    public void setFinalEndEffectorState(EndEffectorState eeState){
        finalEndEffectorState = eeState;
    }

}
