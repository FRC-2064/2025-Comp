package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.RobotSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ClimbSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem;
import frc.robot.Subsystems.Arm.EndEffectorSubsystem.EndEffectorState;
import frc.robot.Subsystems.Arm.WristSubsystem;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.WristConstants;

public class BasicCmd {
    public static final frc.robot.Commands.BasicCmd.ArmCommands ArmCommands = null;
    WristSubsystem wrist;
    ArmSubsystem arm;
    EndEffectorSubsystem endEffector;
    ClimbSubsystem climb;
    RobotSubsystem robot;

    public final ArmCommands armCmd;
    public final EndEffectorCommands eeCmd;
    public final ClimbCommands climbCmd;

    public BasicCmd(WristSubsystem wrist, ArmSubsystem arm, EndEffectorSubsystem endEffector, ClimbSubsystem climb, RobotSubsystem robot) {
        this.wrist = wrist;
        this.arm = arm;
        this.endEffector = endEffector;
        this.climb = climb;
        this.robot = robot;


        this.armCmd = new ArmCommands();
        this.eeCmd = new EndEffectorCommands();
        this.climbCmd = new ClimbCommands();
    }

    public class ArmCommands {
        public Command FrontFeeder = new InstantCommand(robot::armToFeeder);

        public Command FrontL1 = new InstantCommand(() -> {
            arm.setTargetAngle(ArmConstants.ARM_TROUGH_FRONT_ANGLE);
            wrist.setTargetAngle(WristConstants.WRIST_TROUGH_FRONT_ANGLE);
        });

        public Command FrontL2 = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_L2_FRONT_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_L2_FRONT_ANGLE);
                });

        public Command BackL2 = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_L2_BACK_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_L2_BACK_ANGLE);
                });

        public Command BackL3 = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_L3_BACK_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_L3_BACK_ANGLE);
                });

        public Command LowAlgae = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_LOW_ALGAE_REMOVAL_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_LOW_ALGAE_REMOVAL_ANGLE);
                });

        public Command HighAlgae = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_HIGH_ALGAE_REMOVAL_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_HIGH_ALGAE_REMOVAL_ANGLE);
                });

        public Command climbUp = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_CLIMB_UP_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_CLIMB_ANGLE);
                });

        public Command climbDown = new InstantCommand(
                () -> {
                    arm.setTargetAngle(ArmConstants.ARM_CLIMB_DOWN_ANGLE);
                    wrist.setTargetAngle(WristConstants.WRIST_CLIMB_ANGLE);
                });

        public Command toggleArmBrake = new InstantCommand(() -> {
            arm.armToggleCoast();
            wrist.wristToggleCoast();
        });

        public Command groundIntake = new InstantCommand(() -> {
            arm.setTargetAngle(ArmConstants.ARM_GROUND_INTAKE);
            wrist.setTargetAngle(WristConstants.WRIST_GROUND_INTAKE);
        });

    }

    public class EndEffectorCommands {
        public Command PPOuttakeEE = new SequentialCommandGroup(
                new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.OUTTAKING_CORAL)),
                new WaitCommand(0.5),
                new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.STOPPED))
                );
        public Command PPIntakeEE = new AutoIntake(endEffector);
        public Command PPHighEE = new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.REMOVING_HIGH_ALGAE)),
            new WaitCommand(0.5),
            new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.STOPPED))
            );
        public Command PPLowEE = new SequentialCommandGroup(
            new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.REMOVING_LOW_ALGAE)),
            new WaitCommand(0.85),
            new InstantCommand(() -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.STOPPED))
            );
        public Command PPStopEE = new InstantCommand(
                () -> endEffector.setState(EndEffectorSubsystem.EndEffectorState.STOPPED));

        public Command OuttakeEE = new StartEndCommand(
            () -> endEffector.setState(EndEffectorState.OUTTAKING_CORAL),
            () -> endEffector.setState(EndEffectorState.STOPPED),
            endEffector
        );
        public Command IntakeEE = new StartEndCommand(
            () -> endEffector.setState(EndEffectorState.INTAKING_CORAL),
            () -> endEffector.setState(EndEffectorState.STOPPED),
            endEffector
        );
        public Command HighEE = new StartEndCommand(
            () -> endEffector.setState(EndEffectorState.REMOVING_HIGH_ALGAE),
            () -> endEffector.setState(EndEffectorState.STOPPED),
            endEffector
        );
        public Command LowEE = new StartEndCommand(
            () -> endEffector.setState(EndEffectorState.REMOVING_LOW_ALGAE),
            () -> endEffector.setState(EndEffectorState.STOPPED),
            endEffector
        );

        public Command ControlBoardEEIntake = new StartEndCommand(
            robot::runIntakeConfig,
            () -> endEffector.setState(EndEffectorState.STOPPED), 
             robot, endEffector
             );

        public Command ControlBoardEEOuttake = new StartEndCommand(
            robot::runOuttakeConfig,
            () -> endEffector.setState(EndEffectorState.STOPPED), 
             robot, endEffector
             );

        // public Command StopEE = new StartEndCommand(
        //     () -> endEffector.setState(EndEffectorState.STOPPED), 
        //     () -> endEffector.setState(EndEffectorState.STOPPED),
        //     endEffector);
       

    }

    public class ClimbCommands {
        public Command winchIn = new StartEndCommand(
                climb::winchIn,
                climb::winchStop,
                climb);

        public Command toggleClamp = new InstantCommand(climb::toggleClamp);


    }
    
}
