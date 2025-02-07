package frc.robot.ControlBoard;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;

public class ControlBoardHelpers {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("ControlBoard");
    
    public StringEntry feeder;
    public StringEntry scoreLocation;
    public StringEntry reefLocation;
    public IntegerEntry reefLevel;
    public StringEntry bargeCage;
    public BooleanEntry hasScored;
    public StringEntry selectedAuto;
    public BooleanEntry hasCoral;
    public BooleanEntry hasAlgae;
    public BooleanEntry Clamped;
    public BooleanEntry isClimbed;

    public ControlBoardHelpers() {
        feeder = table.getStringTopic("Feeder").getEntry("RIGHT");
        scoreLocation = table.getStringTopic("ScoreLocation").getEntry("REEF");
        reefLocation = table.getStringTopic("Reef/Location").getEntry("A");
        reefLevel = table.getIntegerTopic("Reef/Level").getEntry(0);
        bargeCage = table.getStringTopic("Barge/Cage").getEntry("LEFT");
        hasScored = table.getBooleanTopic("Robot/HasScored").getEntry(false);
        selectedAuto = table.getStringTopic("Robot/SelectedAuto").getEntry("");
        hasCoral = table.getBooleanTopic("Robot/HasCoral").getEntry(false);
        hasAlgae = table.getBooleanTopic("Robot/HasAlgae").getEntry(false);
        Clamped = table.getBooleanTopic("Robot/Clamped").getEntry(false);
        isClimbed = table.getBooleanTopic("Robot/IsClimbed").getEntry(false); 
    }
}
