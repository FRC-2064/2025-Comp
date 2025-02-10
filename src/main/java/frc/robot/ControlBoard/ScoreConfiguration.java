package frc.robot.ControlBoard;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.EndEffectorSubsystem.EndEffectorState;

public class ScoreConfiguration {
    public final Pose2d desiredEndPose;
    public final double finalArmAngle;
    public final double travelArmAngle;
    public final double finalWristAngle;
    public final double travelWristAngle;
    public final EndEffectorState endEffectorState;
    
    public ScoreConfiguration(
        Pose2d desiredEndPose, 
        double finalArmAngle, 
        double travelArmAngle, 
        double finalWristAngle,  
        double travelWristAngle,
        EndEffectorState endEffectorState) {

        this.desiredEndPose = desiredEndPose;
        this.finalArmAngle = finalArmAngle;
        this.travelArmAngle = travelArmAngle;
        this.finalWristAngle = finalWristAngle;
        this.travelWristAngle = travelWristAngle;
        this.endEffectorState = endEffectorState;
    }

}
