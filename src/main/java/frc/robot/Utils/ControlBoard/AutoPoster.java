package frc.robot.Utils.ControlBoard;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class AutoPoster {

    public static void postAutos() {
        File deployDir = Filesystem.getDeployDirectory();

        File autosDir = new File(deployDir, "pathplanner/autos");

        File[] autos = autosDir.listFiles();
        if (autos == null) {
            return;
        }

        String[] autoNames = new String[autos.length];
        for (int i = 0; i < autos.length; i++) {
            autoNames[i] = autos[i].getName().replace(".auto","");
        }

        ControlBoardHelpers.setAutos(autoNames);

    }
    
}
